##===-- HeadlessTrackGenerator.py - HeadlessTrackGenerator -*- py -*-===##
##
## Part of the UGRDV Project, under the UGRACING DRIVERLESS PRIVATE LICENSE.
## See the attached LICENSE.txt file in this projects upper most directory for 
## license information.
##
## Copyright Joseph Agrane © 2023
##
##===----------------------------------------------------------------------===##
###
### \file
### A headless interface to the EUFS Sim random track generator
###
##===----------------------------------------------------------------------===##
#TODO: find out what lax generation is
#TODO: parameterize the number of tracks to generate
from os import path
from rclpy.node import Node
import rclpy
import random
from .TrackGenerator import TrackGenerator as Generator
from .TrackGenerator import GeneratorContext
from .ConversionTools import ConversionTools as Converter
from ament_index_python import get_package_share_directory

class HeadlessTrackGenerator(Node):
    def __init__(self):
        super().__init__("headless_track_generator")
        random.seed(1)
        self.get_logger().info("Starting headless track generator.")
        self.declare_ros_parameters()
        self._generator_values = self.read_ros_parameters()
        self.generate_random_track("joe3012_random_track_test", True)

    def declare_ros_parameters(self):
        self.declare_parameter("component_data_straight", 1.0)
        self.declare_parameter("component_data_constant_turn", 0.7)
        self.declare_parameter("component_data_hairpin_turn", 0.3)

        self.declare_parameter("generator_values_min_straight", 10.0)
        self.declare_parameter("generator_values_max_straight", 80.0)
        self.declare_parameter("generator_values_min_constant_turn", 10.0)
        self.declare_parameter("generator_values_max_constant_turn", 25.0)
        self.declare_parameter("generator_values_min_hairpin", 4.5)
        self.declare_parameter("generator_values_max_hairpin", 10.0)
        self.declare_parameter("generator_values_max_hairpin_pairs", 3)
        self.declare_parameter("generator_values_max_length", 1500.0)
        self.declare_parameter("generator_values_lax_generation", False)
        self.declare_parameter("generator_values_track_width", 3.5)

    def read_ros_parameters(self):
        component_data = {
            "STRAIGHT" : self.get_parameter("component_data_straight").value,
            "CONSTANT_TURN": self.get_parameter("component_data_constant_turn").value,
            "HAIRPIN_TURN" : self.get_parameter("component_data_hairpin_turn").value,
        }
        generator_values = {
            "MIN_STRAIGHT": self.get_parameter("generator_values_min_straight").value,
            "MAX_STRAIGHT": self.get_parameter("generator_values_max_straight").value,
            "MIN_CONSTANT_TURN": self.get_parameter("generator_values_min_constant_turn").value,
            "MAX_CONSTANT_TURN": self.get_parameter("generator_values_max_constant_turn").value,
            "MIN_HAIRPIN": self.get_parameter("generator_values_min_hairpin").value,
            "MAX_HAIRPIN": self.get_parameter("generator_values_max_hairpin").value,
            "MAX_HAIRPIN_PAIRS": self.get_parameter("generator_values_max_hairpin_pairs").value,
            "MAX_LENGTH": self.get_parameter("generator_values_max_length").value,
            "LAX_GENERATION": self.get_parameter("generator_values_lax_generation").value,
            "TRACK_WIDTH": self.get_parameter("generator_values_track_width").value,
            "COMPONENTS": component_data
            }
        return generator_values

    def generate_random_track(self, filename, save_image=False):
        tracks_folder = get_package_share_directory("eufs_tracks")
        images_folder = path.join(tracks_folder, "image")
        generator_values = self._generator_values

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

            if save_image == True: im.save(path.join(images_folder, f"{filename}.png"))

def main(args=None):
    rclpy.init(args=args)
    node = HeadlessTrackGenerator()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
