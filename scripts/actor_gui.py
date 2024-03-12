#!/usr/bin/env python3

import actor_ros.actor_tools as actor_tools  # ACTor specific utility functions
import rospkg  # ROS Package Utilities
from local_file_picker import local_file_picker  # local_file_picker (NiceGUI example)
from nicegui import ui  # NiceGUI library

# ROS I/O -------------------------------------------------------------------------------------------------------------
# NOTE: using a rospy node should be avoided in this python script.
# Since NiceGUI and ROS both requrie the main thread to run, this is a bad idea and will cause event handling errors

# Read ACTor Status - Subscribe to ACTor Status Messages and constantly update using dictdatabase
actor = actor_tools.ActorStatusReader(read_from_redis=True)

# Choose EITHER fake or simulated values for testing purposes:
ui.timer(interval=(1 / 60), callback=actor.redis_callback())  # Update status from database with a timer
# actor.simulate_for_testing()  # Simulate variables for testing purposes

# Path to scripts folder
script_folder = rospkg.RosPack().get_path("actor_ros") + "/igvc_scripts/"


# Configuration and styles for GUI ------------------------------------------------------------------------------------
# NOTE: Add spaces before and after each argument in string format. eg: <space>p-0<space> => " p-0 "
# This helps chaining Tailwind CSS classes or Quasar props. Their arg separator is a whitespace

# COLOR THEME - Quasar props
primary_color_props = " color=blue-8 "
secondary_color_props = " color=light-blue-7 "
accent_color_props = " color=cyan-7 "
text_color_props = " text-color=grey-6 "
button_color_props = " color=grey-8 "
# COLOR THEME - Tailwind classes
text_color_classes = " text-stone-400 "
background_color_classes = " bg-neutral-800 "
# COMMON STYLES - for ease of use and maintainability
footer_card_classes = (
    " shadow-none rounded-none w-full h-24 gap-0 p-0 mx-auto " + text_color_classes + background_color_classes
)
footer_label_classes = " select-none font-bold mx-auto text-xs " + text_color_classes
grid_card_classes = " shadow-none w-80 m-auto gap-2 p-2 border-4 grid " + text_color_classes
button_classes = " w-full h-full m-auto text-bold " + text_color_classes
button_props = " stack push no-caps " + text_color_props + button_color_props


# GUI Design ----------------------------------------------------------------------------------------------------------
ui.page_title("ACTor Web GUI")


# Header --------------------------------------------------------------------------------
with ui.header().classes(replace="row items-center") as header:
    with ui.tabs() as tabs:
        ui.tab("A")
        ui.tab("B")
        ui.tab("C")

# Script Selection and Playback Card
with ui.card() as script_card:
    script_card.classes(grid_card_classes + " grid-cols-3 grid-rows-2")

    async def pick_file() -> None:
        """Open a dialog to select a script file name and load it"""
        # Use local_file_picker (NiceGUI example) to select script
        loaded_script = await local_file_picker(script_folder, multiple=False)

        ui.notify(f"Script selected: {loaded_script[0]}")
        # Change button text to script name
        load_script_button.set_text(f"{loaded_script[0]}")
        # Change button color to green to indicate script loaded
        load_script_button.props("color=light-blue-7 text-color=white")

    # Select Script Button
    load_script_button = ui.button("Choose file", on_click=pick_file, icon="folder")
    load_script_button.classes(button_classes + " col-span-3 row-span-1")
    load_script_button.props(button_props)


# Footer --------------------------------------------------------------------------------
with ui.footer(value=True) as footer:
    footer.classes("shadow-none w-full gap-0 p-0 grid grid-cols-10 grid-rows-1 m-auto")

    # Dashboard (Vehicle Status)

    # Pedals -----
    with ui.card() as pedals_card:
        pedals_card.classes(footer_card_classes + " col-span-2 grid grid-cols-2 grid-rows-4")

        ui.label("PEDALS").classes(footer_label_classes + " col-span-2 row-span-1")
        # Accelerator
        accelerator_slider = (
            ui.slider(min=20, max=80, value=0)
            .props("readonly vertical reverse color=green-10 track-size=10px thumb-size=0px")
            .classes("h-full row-span-3 my-auto pb-2 justify-end")
            .bind_value_from(actor, "accelerator_percent")
        )
        # Brakes
        brakes_slider = (
            ui.slider(min=0, max=100, value=0)
            .props("readonly vertical reverse color=red-10 track-size=10px thumb-size=0px")
            .classes("h-full row-span-3 my-auto pb-2 justify-start")
            .bind_value_from(actor, "brakes_percent")
        )

    # Steering -----
    with ui.card() as steering_card:
        steering_card.classes(footer_card_classes + " col-span-2")

        ui.label("STEERING").classes(footer_label_classes)
        steering_slider = (
            ui.slider(min=-40, max=40, value=-37)
            .props("readonly selection-color=transparent thumb-color=blue-grey-7 thumb-size=20px track-size=10px")
            .classes("w-full m-auto")
            .bind_value_from(actor, "road_angle")
        )
        ui.label("DEG").classes(footer_label_classes).bind_text_from(actor, "road_angle")

    # Auto Pilot -----
    with ui.card() as auto_pilot_card:
        auto_pilot_card.classes(footer_card_classes + " col-span-2")

        ui.label("AUTO PILOT").classes(footer_label_classes)
        auto_pilot_spinner = (
            ui.spinner("bars")
            .props(accent_color_props)
            .classes(button_classes + " p-2")
            .bind_visibility_from(actor, "is_enabled")
        )
        ui.label("ENABLED").classes(footer_label_classes + " my-auto").bind_visibility_from(actor, "is_enabled")
        ui.label("DISABLED").classes(footer_label_classes + " my-auto").bind_visibility_from(actor, "not_is_enabled")
        # NOTE: not_is_enabled is a relational property written in the status node. It is solely used for GUI purposes

    # Speed -----
    with ui.card() as speed_card:
        speed_card.classes(footer_card_classes + " col-span-2")

        ui.label("SPEED").classes(footer_label_classes)
        speed_label = (
            ui.label("04.35")
            .classes("select-none font-bold text-stone-400 text-4xl m-auto")
            .bind_text_from(actor, "speed")
        )
        ui.label("MPH").classes(footer_label_classes)

    # Gear -----
    with ui.card() as gear_card:
        gear_card.classes(footer_card_classes + " col-span-2")

        ui.label("GEAR").classes(footer_label_classes)
        gear_label = ui.label("N").classes("select-none font-bold text-stone-400 text-6xl m-auto")


# Floating area -------------------------------------------------------------------------
# NOTE: needs to be the last element to display on top of all other content
with ui.page_sticky(position="bottom", x_offset=20, y_offset=20):
    # UI functions only - ROS stuff handled separately
    def activate_e_stop():
        # Send E-Stop command to ROS via status class
        actor.send_e_stop()  # TODO: Use EStopManager

        ui.notify("E-STOP ACTIVATED", type="warning", position="center")

        # These should go in a separate thread to always be up-to-date based on the status
        e_stop_button.props("color=dark text-color=positive")
        e_stop_spinner.props("color=positive")

    def reset_e_stop():
        # Send E-Stop reset command to ROS via status class
        actor.send_reset_e_stop()  # TODO: Use EStopManager

        ui.notify("E-STOP RESET", type="positive", position="center")
        e_stop_button.props("color=warning text-color=negative")
        e_stop_spinner.props("color=negative")

    # E-Stop button ----- # TODO: Use EStopManager
    with ui.button(on_click=lambda: activate_e_stop() if not actor.e_stop else reset_e_stop()) as e_stop_button:
        e_stop_button.props(button_props + " color=warning text-color=negative")
        e_stop_button.classes("w-20 h-20 m-auto text-bold text-center")
        e_stop_button.bind_text_from(actor, "e_stop_text")

        e_stop_spinner = (
            ui.spinner("puff", size="20px")
            .props("color=negative")
            .classes("m-auto")
            .bind_visibility_from(actor, "e_stop_heartbeat")
        )


# Run GUI -------------------------------------------------------------------------------------------------------------
# NOTE: Needs to be the last element to run the GUI.
# NOTE: If there are any changes made to this script while it is running, it will restart automatically
ui.run(dark=True, reload=True)
