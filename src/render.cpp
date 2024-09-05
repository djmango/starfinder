#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>


namespace po = boost::program_options;


constexpr char OPT_HELP[] = "help";
constexpr char OPT_FILE[] = "FILE";
constexpr char OPT_DISPLAY_COUNT[] = "display-count";
constexpr char OPT_MIN_RA[] = "min-ra";
constexpr char OPT_MAX_RA[] = "max-ra";
constexpr char OPT_MIN_DEC[] = "min-dec";
constexpr char OPT_MAX_DEC[] = "max-dec";
constexpr char OPT_MAX_MAGNITUDE[] = "max-magnitude";
constexpr char OPT_WIDTH[] = "width";
constexpr char OPT_HEIGHT[] = "height";
constexpr char OPT_OUTPUT[] = "output";


/**
 * \brief   Represents a star with its right ascension, declination, and magnitude.
 */
struct Star {
    const double ra_deg;
    const double de_deg;
    const double mag;

    Star(
                const double ra_deg,
                const double de_deg,
                const double mag
    ) noexcept:
            ra_deg(ra_deg),
            de_deg(de_deg),
            mag(mag)
    {}
};


std::optional<double> parse_field(
        const std::vector<std::string>& record,
        const size_t index,
        const std::string& field_name
) {
    double value;

    try {
        value = std::stod(record.at(index));
    }
    catch (const std::out_of_range&) {
        throw std::runtime_error(
            (
                boost::format("Missing field: %1%") % field_name
            ).str()
        );
    }
    catch (const std::invalid_argument& e) {
        throw std::runtime_error(
            (
                boost::format("Failed to parse %1%. %2%") % field_name % e.what()
            ).str()
        );
    }
    
    return value;
}


double parse_magnitude(const std::vector<std::string>& record) {
    std::optional<double> bt_mag, vt_mag;
    try {
        bt_mag = parse_field(record, 17, "BT magnitude");
        vt_mag = parse_field(record, 19, "VT magnitude");
    }
    catch (const std::runtime_error&) {};

    if (bt_mag) {
        const auto bt = bt_mag.value();
        if (vt_mag) {
            const auto vt = vt_mag.value();
            const auto v_mag = vt - 0.090 * (bt - vt);
            // std::cout << boost::format("Debug: Calculated V_Mag = %1$.3f") % v_mag << std::endl;
            return v_mag;
        } else {
            // std::cout << boost::format("Debug: Using BT_Mag as V_Mag = %1$.3f") % bt << std::endl;
            return bt;
        }
    } else {
        if (vt_mag) {
            const auto vt = vt_mag.value();
            // std::cout << boost::format("Debug: Using VT_Mag as V_Mag = %1$.3f") % vt << std::endl;
            return vt;
        } else {
            throw std::runtime_error("Missing magnitude");
        }
    }
}


Star parse_star_record(const std::vector<std::string>& record) {
    const auto ra = parse_field(record, 24, "RA");
    const auto dec = parse_field(record, 25, "Dec");
    const auto mag = parse_magnitude(record);

    return Star(
        ra.value(),
        dec.value(),
        mag
    );
}


std::vector<Star> read_stars(
        const std::string& path,
        const double min_ra,
        const double max_ra,
        const double min_dec,
        const double max_dec,
        const double max_magnitude
) {
    std::vector<Star> stars;
    std::size_t skipped_rows = 0;
    {
        std::ifstream file(path);
        std::size_t i = 0;
        for (std::string line; std::getline(file, line);) {
            std::vector<std::string> record;
            record.reserve(35);
            boost::split(
                record,
                line,
                boost::is_any_of("|")
            );
            try {
                const auto star = parse_star_record(record);
                if (
                        star.ra_deg >= min_ra
                        &&
                        star.ra_deg <= max_ra
                        &&
                        star.de_deg >= min_dec
                        &&
                        star.de_deg <= max_dec
                        &&
                        star.mag <= max_magnitude
                ) {
                    if ((i % 10000) == 0)
                        std::cout << boost::format("Star %1%: RA=%2%, Dec=%3%, Mag=%4%") % i % star.ra_deg % star.de_deg % star.mag << std::endl;
                    
                    stars.push_back(star);
                }
            }
            catch (const std::runtime_error& e) {
                skipped_rows++;
                if (skipped_rows <= 10) {
                    std::cerr << boost::format("Skipping row %1% due to error: %2%") % i % e.what() << std::endl;
                    std::cerr << boost::format("Problematic row: %1%") % line << std::endl;
                } else if (skipped_rows == 11) {
                    std::cerr << "Further skipped rows will not be printed..." << std::endl;
                }
            }
        }
    }

    std::cout << "Total stars read and filtered: " << stars.size() << std::endl;
    std::cout << "Total rows skipped: " << skipped_rows << std::endl;

    return stars;
};


template <class Clock>
class Stopwatch {
    public:
        Stopwatch();

        float elapsed() const;

    private:
        const typename Clock::time_point start;
};


template <class Clock>
Stopwatch<Clock>::Stopwatch():
        start(Clock::now())
{}


template <class Clock>
float Stopwatch<Clock>::elapsed() const {
    const auto stop = Clock::now();
    return static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count()) / 1000;
}


void render_stars(
        const std::vector<Star>& stars,
        const uint32_t width,
        const uint32_t height,
        const double min_ra,
        const double max_ra,
        const double min_dec,
        const double max_dec,
        cv::OutputArray dst
) {
    dst.create(height, width, CV_8UC1);
    cv::Mat img = dst.getMat();

    // Find the minimum and maximum magnitudes in the dataset
    const auto [min_mag_star, max_mag_star] = std::minmax_element(
        stars.cbegin(),
        stars.cend(),
        [] (const Star& a, const Star& b) {
            return (a.mag < b.mag);
        }
    );
    const auto min_mag = min_mag_star->mag;
    const auto max_mag = max_mag_star->mag;

    std::cout << boost::format("Magnitude range: %1$.3f to %2$.3f") % min_mag % max_mag << std::endl;

    const auto ra_range = max_ra - min_ra;
    const auto dec_range = max_dec - min_dec;
    const auto mag_range = max_mag - min_mag;
    for (const Star& star : stars) {
        const uint32_t x = (star.ra_deg - min_ra) / ra_range * width;
        const uint32_t y = (star.de_deg - min_dec) / dec_range * height;

        if (x < width && y < height) {
            // Inverse the magnitude scale (brighter stars have lower magnitudes)
            const auto normalized_mag = (max_mag - star.mag) / mag_range;

            // Apply a non-linear scaling to emphasize brighter stars
            const uint8_t brightness = std::pow(normalized_mag, 2.5) * 255;

            cv::circle(
                img,
                cv::Point(x, y),
                0,
                cv::Scalar(brightness)
            );
        }
    }
}


int main(int argc, char** argv) {
    po::variables_map vm;
    {
        po::options_description general_options("General options");
        general_options.add_options()
            (OPT_HELP, "print this message")
            (OPT_WIDTH, po::value<uint32_t>()->default_value(800), "Output image width in pixels")
            (OPT_HEIGHT, po::value<uint32_t>()->default_value(600), "Output image height in pixels")
            (OPT_OUTPUT, po::value<std::string>()->default_value("star_map.png"), "Output image file name")
        ;

        po::options_description filter_options("Filter options");
        filter_options.add_options()
            (OPT_DISPLAY_COUNT, po::value<uint32_t>()->default_value(10), "Number of stars to display (0 for all)")
            (OPT_MIN_RA, po::value<double>()->default_value(0), "Minimum Right Ascension (degrees)")
            (OPT_MAX_RA, po::value<double>()->default_value(360), "Maximum Right Ascension (degrees)")
            (OPT_MIN_DEC, po::value<double>()->default_value(-90), "Minimum Declination (degrees)")
            (OPT_MAX_DEC, po::value<double>()->default_value(90), "Maximum Declination (degrees)")
            (OPT_MAX_MAGNITUDE, po::value<double>()->default_value(6), "Maximum visual magnitude (lower is brighter)")
        ;

        po::options_description arguments("Arguments");
        arguments.add_options()
            (OPT_FILE, po::value<std::string>()->default_value("data/tycho2/catalog.dat"), "Path to the Tycho-2 catalog file")
        ;

        po::positional_options_description arguments_positions;
        arguments_positions.add(OPT_FILE, 1);

        po::options_description all_options("All options");
        all_options.add(general_options).add(filter_options).add(arguments);

        po::store(
            po::command_line_parser(argc, argv).options(all_options).positional(arguments_positions).run(),
            vm
        );
        po::notify(vm);

        if (vm.count(OPT_HELP) != 0) {
            std::cout << "render [options]";
            std::cout << ' ' << OPT_FILE;
            std::cout << std::endl << std::endl;
            std::cout << arguments << std::endl;
            std::cout << general_options << std::endl;
            std::cout << filter_options << std::endl;
            return -1;
        }
    }


    std::cout << boost::format("Reading stars from: %1%") % vm[OPT_FILE].as<std::string>() << std::endl;
    std::cout << boost::format("RA range: %1% to %2%") % vm[OPT_MIN_RA].as<double>() % vm[OPT_MAX_RA].as<double>() << std::endl;
    std::cout << boost::format("Dec range: %1% to %2%") % vm[OPT_MIN_DEC].as<double>() % vm[OPT_MAX_DEC].as<double>() << std::endl;
    std::cout << boost::format("Max magnitude: %1%") % vm[OPT_MAX_MAGNITUDE].as<double>() << std::endl;

    const Stopwatch<std::chrono::high_resolution_clock> read_start;
    const auto stars = read_stars(
        vm[OPT_FILE].as<std::string>(),
        vm[OPT_MIN_RA].as<double>(),
        vm[OPT_MAX_RA].as<double>(),
        vm[OPT_MIN_DEC].as<double>(),
        vm[OPT_MAX_DEC].as<double>(),
        vm[OPT_MAX_MAGNITUDE].as<double>()
    );
    const auto read_duration = read_start.elapsed();

    std::cout << "Time taken to read and filter stars: " << read_duration << std::endl;
    std::cout << "Total stars after filtering: " << stars.size() << std::endl;
    std::cout << std::endl;

    {
        const uint32_t display_count = vm[OPT_DISPLAY_COUNT].as<uint32_t>();
        std::cout << boost::format("First %1% stars:") % display_count << std::endl;
        uint32_t i = 0;
        for (const auto& star : stars) {
            if (i >= display_count && display_count != 0)
                break;
            std::cout << boost::format("Star %1%: RA=%2$.2f, Dec=%3$.2f, Mag=%4$.2f") % i % star.ra_deg % star.de_deg % star.mag << std::endl;
            i++;
        }
    }

    cv::Mat img;
    const Stopwatch<std::chrono::high_resolution_clock> render_start;
    render_stars(
        stars,
        vm[OPT_WIDTH].as<uint32_t>(),
        vm[OPT_HEIGHT].as<uint32_t>(),
        vm[OPT_MIN_RA].as<double>(),
        vm[OPT_MAX_RA].as<double>(),
        vm[OPT_MIN_DEC].as<double>(),
        vm[OPT_MAX_DEC].as<double>(),
        img
    );
    cv::imwrite(vm[OPT_OUTPUT].as<std::string>(), img);
    const auto render_duration = render_start.elapsed();

    std::cout << "Time taken to render and save image: " << render_duration << std::endl;
    std::cout << "Image saved as: " << vm[OPT_OUTPUT].as<std::string>() << std::endl;
    std::cout << "Total time elapsed: " << read_start.elapsed() << std::endl;

    return 0;
}