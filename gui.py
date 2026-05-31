import dearpygui.dearpygui as dpg

STATE_COLORS = {
    "in_queue": (255, 200, 60, 255),
    "queue_end": (255, 120, 60, 255),
    "queue_start": (255, 255, 60, 255),
}


def setup_ui(W, H, sim):

    dpg.create_context()
    dpg.create_viewport(title="Sim", width=W, height=H)

    # Setup Sim UI
    with dpg.window(label="Sim", no_scrollbar=True, tag="sim_window"):
        with dpg.drawlist(width=W, height=H, tag="canvas"):
            dpg.add_draw_layer(tag="floor")
            dpg.add_draw_layer(tag="agents")

    def _param_cb(sender, value, key):
        sim.params[key] = value

    SLIDERS = [
        ("desired_speed", 0, 10),
        ("reaction_time", 0.25, 20),
        ("influence_radius", 0, 200),
        ("proximity_threshold", 0, 200),
        ("A", 0, 10),
        ("B", 1, 60),                 
        ("containment_strength", 0, 0.01),
        ("corridor_width", 0, 300),
        ("follow_gap", 0, 100),
        ("wait_time", 0, 600),
        ("spawn_spread", 0, 100),
    ]

    # Setup Param UI
    with dpg.window(label="Parameters", width=400, pos=(W - 440, 20)):
        for key, lo, hi in SLIDERS:
            dpg.add_slider_float(
                label=key, default_value=sim.params[key], min_value=lo, max_value=hi, callback=_param_cb, user_data=key
            )

    # Draw floor
    dpg.draw_rectangle((0, 0), (W, H), fill=(28, 32, 38, 255), parent="floor")
    dpg.draw_circle(sim.params["end_point"], 7, fill=STATE_COLORS["queue_end"], color=(0, 0, 0, 0), parent="floor")
    dpg.draw_circle(sim.params["start_point"], 7, fill=STATE_COLORS["queue_start"], color=(0, 0, 0, 0), parent="floor")

    # On click spawn agent at click location
    def on_click():
        if not dpg.is_item_hovered("canvas"):
            return
        x, y = dpg.get_drawing_mouse_pos()
        sim.spawn(x, y)

    with dpg.handler_registry():
        dpg.add_mouse_click_handler(button=dpg.mvMouseButton_Left, callback=on_click)

    # Display viewport
    dpg.setup_dearpygui()
    dpg.set_primary_window("sim_window", True)
    dpg.show_viewport()


def redraw(agents, end):
    dpg.delete_item("agents", children_only=True)
    for a in agents:
        dpg.draw_circle(
            (a.x, a.y),
            7,
            fill=STATE_COLORS["in_queue"],
            color=(0, 0, 0, 0),
            parent="agents",
        )
