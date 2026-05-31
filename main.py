import dearpygui.dearpygui as dpg

dpg.create_context()
dpg.create_viewport(title='RSSI Simulator', width=600, height=400)

with dpg.window(label="Main Window"):
    dpg.add_text("Welcome to the RSSI Simulator!")
    dpg.add_button(label="Start Simulation", callback=lambda: print("Simulation started!"))
    dpg.add_button(label="Stop Simulation", callback=lambda: print("Simulation stopped!"))

dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()