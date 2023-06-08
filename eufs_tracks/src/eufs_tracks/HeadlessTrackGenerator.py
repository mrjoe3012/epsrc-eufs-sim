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
import rclpy, random, time
from .TrackGenerator import TrackGenerator as Generator
from .TrackGenerator import GeneratorContext
from .ConversionTools import ConversionTools as Converter
from ament_index_python import get_package_share_directory

class HeadlessTrackGenerator(Node):
    def __init__(self):
        """
        This node is responsible for interfacing with the
        EUFS track generator and generating random tracks.
        """
        super().__init__("headless_track_generator")
        self.get_logger().info("Starting headless track generator.")
        self._declare_ros_parameters()
        self._generator_values, self._node_params = self._read_ros_parameters()
        self._initialise_rng(self._node_params["use_custom_seed"], self._node_params["custom_seed"])

    def _declare_ros_parameters(self):
        """
        This function declares all of the parameters that it expects
        to receive from ROS.
        """
        self.declare_parameter("use_custom_seed", False)
        self.declare_parameter("custom_seed", 0)

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

    def _read_ros_parameters(self):
        """
        Reads expected ROS parameters.

        :returns: Tuple (2,) in which the first element contains all parameters to be consumed
        by the track generator and the second element contains any other parameters.
        """
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
        other_params = {
            "use_custom_seed" : self.get_parameter("use_custom_seed").value,
            "custom_seed" : self.get_parameter("custom_seed").value
        }
        return generator_values, other_params

    def _initialise_rng(self, use_custom_seed: bool, custom_seed = 0):
        """
        Seed the random number generator either with system time or
        with a custom seed.
        
        :param use_custom_seed: Whether or not to use the provided custom seed.
        :param custom_seed: The custom seed to use.
        """
        if use_custom_seed == True:
            random.seed(custom_seed)
            self._rng_seed = custom_seed
        else:
            t = int(time.time())
            random.seed(t)
            self._rng_seed = t
 
    def _generate_random_track(self, filename: str, save_image=False):
        """
        This function interfaces with the EUFS random track generator
        to generate a random track and the associated files.
        
        :param filename: The name of the output files to be generated, minus the extension.
        :param save_image: Whether or not the image file should be saved.
        """
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
