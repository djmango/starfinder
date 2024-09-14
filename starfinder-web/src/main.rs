use actix_web::{get, web, HttpResponse, Result};
use image::ImageFormat;
use serde::Deserialize;
use shuttle_actix_web::ShuttleActixWeb;
use starfinder::parse_and_render::read_and_render;

#[get("/")]
async fn hello_world() -> &'static str {
    "Hello World!"
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
    let image = read_and_render(
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
    .map_err(|e| actix_web::error::ErrorInternalServerError(e))?;

    let mut buffer = Vec::new();
    image
        .write_to(&mut std::io::Cursor::new(&mut buffer), ImageFormat::Png)
        .map_err(actix_web::error::ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().content_type("image/png").body(buffer))
}

#[shuttle_runtime::main]
async fn main() -> ShuttleActixWeb<impl FnOnce(&mut web::ServiceConfig) + Send + Clone + 'static> {
    let config = move |cfg: &mut web::ServiceConfig| {
        cfg.service(hello_world).service(render);
    };

    Ok(config.into())
}
