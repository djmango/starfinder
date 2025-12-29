use actix_files::NamedFile;
use actix_web::{get, web, App, HttpResponse, HttpServer, Result};
use image::ImageFormat;
use serde::Deserialize;
use starfinder::parse_and_render::read_and_render;
use std::path::PathBuf;
use tracing::info;

#[get("/")]
async fn index() -> Result<NamedFile> {
    Ok(NamedFile::open(PathBuf::from("static/index.html"))?)
}

#[derive(Deserialize)]
struct RenderParams {
    center_ra: f64,
    center_dec: f64,
    roll: f64,
    fov_w: f64,
    fov_h: f64,
    max_magnitude: f64,
    width: u32,
    height: u32,
    fov_max: f64,
}

#[get("/render")]
async fn render(params: web::Query<RenderParams>) -> Result<HttpResponse> {
    let image = web::block(move || {
        read_and_render(
            params.center_ra,
            params.center_dec,
            params.roll,
            params.fov_w,
            params.fov_h,
            params.max_magnitude,
            params.width,
            params.height,
            params.fov_max,
        )
    })
    .await
    .map_err(|e| actix_web::error::ErrorInternalServerError(e))?
    .map_err(|e| actix_web::error::ErrorInternalServerError(e))?;

    let mut buffer = Vec::new();
    image
        .write_to(&mut std::io::Cursor::new(&mut buffer), ImageFormat::Png)
        .map_err(actix_web::error::ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().content_type("image/png").body(buffer))
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    tracing_subscriber::fmt::init();

    let bind_addr = std::env::var("BIND_ADDR").unwrap_or_else(|_| "0.0.0.0:8080".to_string());
    info!("Starting server at http://{}", bind_addr);

    HttpServer::new(|| {
        App::new()
            .service(index)
            .service(render)
            .service(actix_files::Files::new("/static", "static/").index_file("index.html"))
    })
    .bind(&bind_addr)?
    .run()
    .await
}
