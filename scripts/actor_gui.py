#!/usr/bin/env python3

import sys
import time  # Time library

import actor_ros.actor_tools as actor_tools  # ACTor specific utility functions
import numpy as np
import rospkg  # ROS Package Utilities
from nicegui import ui  # NiceGUI library
from PIL import Image

# ROS I/O -------------------------------------------------------------------------------------------------------------
# NOTE: using a rospy node should be avoided in this python script.
# Since NiceGUI and ROS both requrie the main thread to run, this is a bad idea and will cause event handling errors

# Read ACTor Status - Subscribe to ACTor Status Messages and constantly update using Redis key value store
if sys.argv[1].lower() == "true":  # Simulated Values -------------------------------
    print("Setting up GUI for simulation...")
    # NOTE: For now, only speed and road angle are updated using twist input
    status = actor_tools.ActorStatusReader(read_from_redis=True)  # add simulate_for_testing=True to test gui elements
    ui.timer(interval=(1), callback=lambda: status())  # Update status from simulated values
else:  # Use Redis to get ACTor Status msgs -----------------------------------------
    print("Setting up GUI for Real Vehicle...")
    status = actor_tools.ActorStatusReader(read_from_redis=True)
    ui.timer(interval=(1 / 60), callback=lambda: status())  # Update status from database with a timer

# GUI Support -----------------------------------
store = actor_tools.RedisReader()
ui.timer(interval=(1 / 30), callback=lambda: store())  # Update store with a timer

# E-STOP ----------------------------------------
estop = actor_tools.EStopManager()

# Script Player ---------------------------------
# Using rospkg to get the full path to the directory containing the scripts
script_player = actor_tools.ScriptPlayer(rospkg.RosPack().get_path("actor_ros") + "/scripts/igvc_scripts/")
script_player.load_files()

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
grid_card_classes = " shadow-none m-auto gap-2 p-2 border-4 grid " + text_color_classes
button_classes = " w-full h-full m-auto text-bold " + text_color_classes
button_props = " stack push no-caps " + text_color_props + button_color_props
image_props = " loading=eager no-transition no-spinner fit=scale-down "


# GUI Design ----------------------------------------------------------------------------------------------------------
ui.page_title("ACTor Web UI")
ui.add_css("""
    :root {
        --nicegui-default-padding: 0.5rem;
        --nicegui-default-gap: 0.5rem;
    }
""")


# Header --------------------------------------------------------------------------------
with ui.header().classes(replace="row items-center") as header:
    with ui.tabs() as tabs:
        ui.tab("A")
        ui.tab("B")
        ui.tab("C")

with ui.column().classes("w-1/2"):
    # Script Selection and Playback Card
    with ui.card() as script_card:
        script_card.classes(grid_card_classes + " w-full grid-cols-4 grid-rows-2")

        async def update_list():
            """Update the dropdown list of script files"""
            ui.notify(script_player.load_files(), type="positive", position="top", timeout=2000)
            file_select_dropdown.options = script_player.file_list

        async def select_file(filename: str) -> None:
            """Select a script file name and load it"""

            ui.notify(f"Script selected: {script_player.selected_file}", position="top", timeout=2000)
            scrolling_log_area.clear()
            with scrolling_log_area:
                ui.label(f" Route Ready: '{script_player.selected_file}' ")

        async def handle_file() -> None:
            """Execute the selected file in a separate process"""

            if script_player.is_running:  # stop the script
                ui.notify(script_player.stop_script(), type="negative", position="top", timeout=2000)
                temp_text = """
                
                **********************
                *** SCRIPT STOPPED ***
                **********************
                
                """
                print(temp_text)

            else:  # start the script
                scrolling_log_area.clear()
                global text_buffer
                text_buffer = []
                ui.notify(script_player.execute(), type="positive", position="top", timeout=2000)

        # Dropdown Menu -----
        file_select_dropdown = (
            ui.select(
                options=script_player.file_list,
                with_input=True,
                clearable=True,
                on_change=lambda e: select_file(e.value),  # Event object with the selected value
            )
            .classes("col-span-4 row-span-1")
            .bind_value(script_player, "selected_file")
        )

        # Reload Button -----
        reload_button = (
            ui.button("Reload", on_click=update_list)
            .classes(button_classes + " col-span-2 row-span-1")
            .props(button_props)
        )

        # Start/Stop Button -----
        run_button = (
            ui.button("Start", on_click=handle_file)
            .classes(button_classes + " col-span-2 row-span-1")
            .props(button_props)
            .bind_text_from(script_player, "is_running", lambda running: "Stop" if running else "Start")
        )

    with ui.card() as log_card:
        log_card.classes(grid_card_classes + " w-full")

        global text_buffer  # Used as Subset of script_player.output_text to keep track of newly appended lines
        text_buffer = []

        def add_label_on_change():
            """Add a new label to scroll area when output_text changes"""
            global text_buffer

            length_difference = len(script_player.output_text) - len(text_buffer)
            if length_difference > 0:  # Check if output_text has newly appended lines
                with scrolling_log_area:
                    for i in range(-length_difference, 0):
                        text = script_player.output_text[i]  # Get the newly appended line
                        text_buffer.append(text)
                        ui.label(text).classes("font-mono leading-none whitespace-pre")
                scrolling_log_area.scroll_to(percent=1.0, duration=0.1)  # Scroll to bottom

        scrolling_log_area = ui.scroll_area().classes("gap-0 show-scrollbar")
        ui.timer(interval=(1 / 60), callback=lambda: add_label_on_change())  # Update log text

        # log_area = (
        #     ui.textarea()
        #     .bind_value_from(script_player, "output_text", backward=lambda x: "\n".join(x))
        #     .classes("h-full overflow-scroll")
        #     .props("readonly dense")
        # )
        # log_area.visible = False

    image_1 = ui.image().props(image_props).bind_source_from(store, "image_1")
    image_2 = ui.image().props(image_props).bind_source_from(store, "image_2")
    image_3 = ui.image().props(image_props).bind_source_from(store, "image_3")
    image_4 = ui.image().props(image_props).bind_source_from(store, "image_4")


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
            .bind_value_from(status, "accelerator_percent")
        )
        # Brakes
        brakes_slider = (
            ui.slider(min=0, max=100, value=0)
            .props("readonly vertical reverse color=red-10 track-size=10px thumb-size=0px")
            .classes("h-full row-span-3 my-auto pb-2 justify-start")
            .bind_value_from(status, "brakes_percent")
        )

    # Steering -----
    with ui.card() as steering_card:
        steering_card.classes(footer_card_classes + " col-span-2")

        ui.label("STEERING").classes(footer_label_classes)
        steering_slider = (
            ui.slider(min=-40, max=40, value=-37)
            .props("readonly selection-color=transparent thumb-color=blue-grey-7 thumb-size=20px track-size=10px")
            .classes("w-full m-auto")
            .bind_value_from(status, "road_angle")
        )
        ui.label("DEG").classes(footer_label_classes).bind_text_from(status, "road_angle")

    # Auto Pilot -----
    with ui.card() as auto_pilot_card:
        auto_pilot_card.classes(footer_card_classes + " col-span-2")

        ui.label("AUTO PILOT").classes(footer_label_classes)
        auto_pilot_spinner = (
            ui.spinner("bars")
            .props(accent_color_props)
            .classes(button_classes + " p-2")
            .bind_visibility_from(status, "is_enabled")
        )
        ui.label("ENABLED").classes(footer_label_classes + " my-auto").bind_visibility_from(status, "is_enabled")
        ui.label("DISABLED").classes(footer_label_classes + " my-auto").bind_visibility_from(
            status, "is_enabled", backward=lambda state: not state
        )

    # Speed -----
    with ui.card() as speed_card:
        speed_card.classes(footer_card_classes + " col-span-2")

        ui.label("SPEED").classes(footer_label_classes)
        speed_label = (
            ui.label("04.35")
            .classes("select-none font-bold text-stone-400 text-4xl m-auto")
            .bind_text_from(status, "speed")
        )
        ui.label("MPH").classes(footer_label_classes)

    # Gear -----
    with ui.card() as gear_card:
        gear_card.classes(footer_card_classes + " col-span-2")

        ui.label("GEAR").classes(footer_label_classes)
        gear_label = ui.label("N").classes("select-none font-bold text-stone-400 text-6xl m-auto")
        gear_label.bind_text_from(status, "gear", backward=lambda gear: gear[0])  # Gear Initial


# Floating area -------------------------------------------------------------------------
# NOTE: needs to be the last element to display on top of all other content
with ui.page_sticky(position="bottom", x_offset=20, y_offset=20):
    # UI functions only - ROS stuff handled separately
    async def activate_estop():
        """Activate E-Stop using EStopManager and notify user"""
        estop()  # Using EStopManager
        ui.notify("E-STOP ACTIVATED", type="warning", position="center", timeout=3000)

    async def reset_estop():
        """Reset E-Stop using EStopManager and notify user"""
        estop.reset()  # Using EStopManager
        ui.notify("E-STOP RESET", type="positive", position="center", timeout=3000)

    def check_estop_state():
        """Check E-Stop state using status.estop_state and update button accordingly"""
        if status.estop_state:  # Active
            estop_button.props("color=dark text-color=positive")
            estop_spinner.props("color=positive")
        else:  # Inactive
            estop_button.props("color=warning text-color=negative")
            estop_spinner.props("color=negative")

    # E-Stop button -----
    with ui.button(on_click=lambda: activate_estop() if not status.estop_state else reset_estop()) as estop_button:
        estop_button.props(button_props + " color=warning text-color=negative")
        estop_button.classes("w-20 h-20 m-auto text-bold text-center")
        estop_button.bind_text_from(status, "estop_state", backward=lambda state: "RESET" if state else "STOP")

        estop_spinner = (
            ui.spinner("puff", size="20px")
            .props("color=negative")
            .classes("m-auto")
            .bind_visibility_from(status, "estop_heartbeat")
        )

    ui.timer(interval=(1 / 20), callback=lambda: check_estop_state())  # Check E-Stop state with a timer


# Run GUI -------------------------------------------------------------------------------------------------------------
# NOTE: Needs to be the last element to run the GUI.
ui.run(dark=True, show=False)
