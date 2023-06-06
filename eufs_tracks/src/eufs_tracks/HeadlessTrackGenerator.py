from os import path
from .TrackGenerator import TrackGenerator as Generator
from .TrackGenerator import GeneratorContext
from .ConversionTools import ConversionTools as Converter
from ament_index_python import get_package_share_directory

def generate_random_track(filename, show_image=False, save_image=False):
    tracks_folder = get_package_share_directory("eufs_tracks")
    is_lax_generator = False
    component_data = {
        "STRAIGHT" : 1.0,
        "CONSTANT_TURN" : 0.7,
        "HAIRPIN_TURN" : 0.3,
    }
    generator_values = {
            "MIN_STRAIGHT": 10.0,
            "MAX_STRAIGHT": 80.0,
            "MIN_CONSTANT_TURN": 10.0,
            "MAX_CONSTANT_TURN": 25.0,
            "MIN_HAIRPIN": 4.5,
            "MAX_HAIRPIN": 10.0,
            "MAX_HAIRPIN_PAIRS": 3,
            "MAX_LENGTH": 1500.0,
            "LAX_GENERATION": is_lax_generator,
            "TRACK_WIDTH": 3.5,
            "COMPONENTS": component_data
        }

    def failure_function():
        print("Track generator has failed.")

    with GeneratorContext(generator_values, failure_function):
        print("Starting track generation")

        xys, twidth, theight = Generator.generate()

        im = Converter.convert(
            "comps",
            "csv",
            filename,
            params={
                "track data": (xys, twidth, theight)
            }
        )

        csv_path = path.join(
            tracks_folder,
            f"csv/{filename}.csv"
        )

        Converter.convert(
            "csv",
            "ALL",
            csv_path,
            params={"noise" : 0.001}
        )

        print("Track gen complete")

        if show_image == True: im.show()
        if save_image == True: im.save(f"{filename}.png")

def main():
    for i in range(10):
        generate_random_track("headless_" + str(i), save_image=True)

if __name__ == "__main__":
    main()
