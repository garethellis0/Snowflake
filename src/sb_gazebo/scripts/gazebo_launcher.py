#!/usr/bin/env python

# TODO: remove unused imports

# ROS Imports
import rospy
import rospkg
import roslaunch

# General Imports
import os
import wx  # used for GUI stuff

class GazeboLauncher():
    """
    Contains all the functionality associated with running gazebo worlds with robots
    """

    # The path to the robot model folder
    robot_models_path = ""

    # The path to the world model folder
    world_models_path = ""

    # A RosPack instance used to query various ROS things
    # (such as the path to various packages)
    rospack = None

    # TODO: Better comment - what are these parts?
    # The package that contains all the relevant parts to launch Gazebo
    gazebo_model_package_name = ""

    def __init__(self, gazebo_model_package_name, subpath_to_robot_models, subpath_to_world_models):
        """
        :param gazebo_model_package_name: the name of the package containing the robot and world models
        :param subpath_to_robot_models: The path to the robot model folder under the `model_package`
        :param subpath_to_world_models: The path to the world model folder under the `model_package`
        """

        self.rospack = rospkg.RosPack()

        # Make sure the gazebo package we were given exists
        package_names = self.rospack.list()
        if gazebo_model_package_name not in package_names:
            rospy.signal_shutdown("Given package \"" + gazebo_model_package_name + "\" not found")
        else:
            self.gazebo_model_package_name = gazebo_model_package_name
            self.model_package_path = self.rospack.get_path(model_package_name)

        # Make sure that the robot models path we were given exists
        self.robot_models_path = os.path.join(self.model_package_path, subpath_to_robot_models)
        if not os.path.isdir(self.robot_models_path):
            rospy.signal_shutdown("Given robot models path \"" + self.robot_models_path + "\" not found")

        # Make sure that the world models path we were given exists
        self.world_models_path = os.path.join(self.model_package_path, subpath_to_world_models)
        if not os.path.isdir(self.world_models_path):
            rospy.signal_shutdown("Given world models path \"" + self.world_models_path + "\" not found")

    def getRobotNames(self):
        """
        :return: The names of all the robots in the robot models folder
        """
        # Return all the folders in the robot models directory
        return next(os.walk(self.robot_models_path))[1]

    def getWorldNames(self):
        """
        :return: The names of all the worlds in the world models folder
        """
        # Return the names of all the world files in the world models folder
        world_names = []
        for file in os.listdir(self.world_models_path):
            filename, extension = os.path.splitext(file)
            if extension == ".world":
                world_names += [filename]
        return world_names

    # TODO: Add option to set initial robot location?
    def runGazebo(self, world_name, robot_name):
        """
        Tries to find the given world and robot and run Gazebo with them
        :param world_name: the name of the world to run
        :param robot_name: the name of the robot to place in the world
        :return:
        """

        # TODO: Actually run Gazebo

        print("Ran gazebo!")


class GazeboLauncherGui(wx.Frame):
    """
    The GUI for running Gazebo

    Extend `wx.Frame`.
    """

    # The parent of this frame
    parent = None

    # A `GazeboLauncher` that we can use for running things
    # and getting information (like a list of robot models and worlds) to display
    gazebo_launcher = None

    # The dropdown menus that will hold the choices the user inputs
    # these are scoped to the class so callback functions (for example, for buttons)
    # can see them
    world_dropdown = None
    robot_dropdown = None

    def __init__(self, parent, id, title, gazebo_launcher):
        """
        :param parent: the parent of this frame
        :param id: the id of this frame
        :param title: the name of the frame (displayed at the top of the window)
        :param gazebo_launcher: a `GazeboLauncher` that we will use for running things
        and getting information (like a list of robot models and worlds) to display
        """

        wx.Frame.__init__(self, parent, id, title)

        self.parent = parent
        self.rospack = rospkg.RosPack()
        self.gazebo_launcher = gazebo_launcher

        # Start the GUI
        self.initialize()


    def initialize(self):
        """
        Sets up the window with all the GUI elements
        """
        # TODO: Add some spacing around the elements
        # The main vertical layout
        vertical_box = wx.BoxSizer(wx.VERTICAL)

        # The label for the world
        world_label = wx.StaticText(self, -1, "Choose a world:")
        vertical_box.Add(world_label, flag=wx.EXPAND|wx.LEFT|wx.RIGHT)

        # The drop-down menu to choose a world
        world_names = self.gazebo_launcher.getWorldNames()
        self.world_dropdown = wx.Choice(self, -1, choices=world_names)
        vertical_box.Add(self.world_dropdown, flag=wx.EXPAND|wx.LEFT|wx.RIGHT)

        # The label for the robot
        robot_label = wx.StaticText(self, -1, "Choose a robot:")
        vertical_box.Add(robot_label, flag=wx.EXPAND|wx.LEFT|wx.RIGHT)

        # The drop-down menu to choose a robot
        robot_names = self.gazebo_launcher.getRobotNames()
        self.robot_dropdown = wx.Choice(self, -1, choices=robot_names)
        vertical_box.Add(self.robot_dropdown, flag=wx.EXPAND|wx.LEFT|wx.RIGHT)

        # The button to run gazebo with the selected choices
        run_button = wx.Button(self, -1, "Run Gazebo")
        vertical_box.Add(run_button, flag=wx.EXPAND|wx.LEFT|wx.RIGHT)

        # Add a binding for the callback when the run button is pressed
        self.Bind(wx.EVT_BUTTON, self.onRun)

        self.SetSizerAndFit(vertical_box)
        self.Show(True)

    def onRun(self, event):
        # Get the index for the users chosen world and robot
        world_selection_index = self.world_dropdown.GetCurrentSelection()
        robot_selection_index = self.robot_dropdown.GetCurrentSelection()

        # Check that we've selected a world
        if world_selection_index == -1:
            err_dialog = wx.MessageDialog(self, "Please select a world!")
            err_dialog.ShowModal()
            return

        # Check that we've selected a robot
        if robot_selection_index == -1:
            err_dialog = wx.MessageDialog(self, "Please select a robot!")
            err_dialog.ShowModal()
            return

        # Get the string values for the users chosen world and robot
        world_name = self.world_dropdown.GetString(world_selection_index)
        robot_name = self.robot_dropdown.GetString(robot_selection_index)

        # Run Gazebo
        self.gazebo_launcher.runGazebo(world_name, robot_name)

        # TODO: Delete me
        print("Hit run!")


def getParamWithDefault(param_name, default_value):
    """
    :param param_name: the name of the param we're looking for
    :return: the value of the param (`default_value` if param not found)
    """
    if rospy.has_param(param_name):
        return rospy.get_param(param_name)
    else:
        rospy.logwarn("No value for param: \"" + param_name +
                      "\" given, defaulting to \"" + default_value + "\"")
        return default_value


if __name__=="__main__":

    # Get RosParams

    # The package that contains all the gazebo models for robots and worlds
    model_package_name = getParamWithDefault("model_package_name", "sb_gazebo")

    # The path to the robot model folder under the `model_package`
    subpath_to_robot_models_folder = getParamWithDefault("robot_model_path", "robots/models")

    # The path to the world model folder under the `model_package`
    subpath_to_world_models_folder = getParamWithDefault("world_models_path", "worlds")

    # Setup the launcher
    gazebo_launcher = GazeboLauncher(
        model_package_name, subpath_to_robot_models_folder, subpath_to_world_models_folder)

    # Setup the app and window
    app = wx.App()
    window_name = "Snowbots Gazebo Launcher"
    frame = GazeboLauncherGui(None, -1, window_name, gazebo_launcher)

    # Run the app
    app.MainLoop()
