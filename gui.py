import dearpygui.dearpygui as dpg

STATE_COLORS = {
    "in_queue": (255, 200, 60, 255),
    "queue_end": (255, 120, 60, 255),
    "queue_start": (255, 255, 60, 255),
    "wall_segment": (200, 200, 200, 255),
    "wall_point": (200, 200, 200, 255),
}

SLIDERS = [
    ("desired_speed", 0, 10),
    ("reaction_time", 0.25, 20),
    ("influence_radius", 0, 200),
    ("proximity_threshold", 0, 200),
    ("A", 0, 10),
    ("B", 1, 60),
    ("A_wall", 0, 10),
    ("B_wall", 1, 60),
    ("follow_gap", 0, 100),
    ("wait_time", 0, 600),
    ("spawn_spread", 0, 100),
    ("head_drive_multiplier", 1.0, 5.0),
    ("group_drive_multiplier", 1.0, 5.0),
]

gui_state = {
    "tool": "spawn",
    "wall_in_progress": None,
    "show_walls": True,
}


def setup_ui(W, H, sim):

    dpg.create_context()
    dpg.create_viewport(title="Sim", width=W, height=H)

    # Setup Sim UI
    with dpg.window(label="Sim", no_scrollbar=True, tag="sim_window"):
        with dpg.drawlist(width=W, height=H, tag="canvas"):
            dpg.add_draw_layer(tag="floor")
            dpg.add_draw_layer(tag="walls")
            dpg.add_draw_layer(tag="agents")

    def _param_cb(sender, value, key):
        sim.params[key] = value

    def toggle_walls(sender, value):
        gui_state["show_walls"] = value
        dpg.configure_item("walls", show=value)

    def clear_walls(self):
        sim.clear_walls()
        dpg.delete_item("walls", children_only=True)
        gui_state["wall_in_progress"] = None

    def clear_queue(self):
        sim.clear_queue()
        dpg.delete_item("agents", children_only=True)

    # Setup Param UI
    with dpg.window(label="Parameters", width=400, pos=(W - 440, 20)):
        for key, lo, hi in SLIDERS:
            dpg.add_slider_float(
                label=key, default_value=sim.params[key], min_value=lo, max_value=hi, callback=_param_cb, user_data=key
            )
        dpg.add_radio_button(
            items=["Spawn", "Wall"],
            default_value="Spawn",
            callback=lambda s, v: gui_state.__setitem__("tool", v.lower()),
        )
        dpg.add_checkbox(
            label="Show Walls",
            default_value=gui_state["show_walls"],
            callback=lambda s, v: toggle_walls(s, v)
        )
        dpg.add_button(label="Clear Walls", callback=clear_walls)
        dpg.add_button(label="Clear Queue", callback=clear_queue)

    # Draw floor
    dpg.draw_rectangle((0, 0), (W, H), fill=(28, 32, 38, 255), parent="floor")
    dpg.draw_circle(sim.params["end_point"], 7, fill=STATE_COLORS["queue_end"], parent="floor")
    dpg.draw_circle(sim.params["start_point"], 7, fill=STATE_COLORS["queue_start"], parent="floor")

    # On click spawn agent at click location
    def on_click():
        if not dpg.is_item_hovered("canvas"):
            return
        x, y = dpg.get_drawing_mouse_pos()

        # Spawn agent
        if gui_state["tool"] == "spawn":
            sim.spawn(x, y)

        # Draw wall
        elif gui_state["tool"] == "wall":
            if not gui_state["wall_in_progress"]:
                gui_state["wall_in_progress"] = (x, y)
                dpg.draw_circle((x, y), 3, fill=STATE_COLORS["wall_point"], parent="walls")
            else:
                pt1, pt2 = gui_state["wall_in_progress"], (x, y)
                sim.walls.append((pt1, pt2))
                dpg.draw_line((pt1[0], pt1[1]), (pt2[0], pt2[1]), color=STATE_COLORS["wall_segment"], parent="walls")
                dpg.draw_circle((x, y), 3, fill=STATE_COLORS["wall_point"], parent="walls")
                gui_state["wall_in_progress"] = None

    with dpg.handler_registry():
        dpg.add_mouse_click_handler(button=dpg.mvMouseButton_Left, callback=on_click)

    # Display viewport
    dpg.setup_dearpygui()
    dpg.set_primary_window("sim_window", True)
    dpg.show_viewport()

def redraw(agents, walls):
    dpg.delete_item("agents", children_only=True)
    for a in agents:
        dpg.draw_circle(
            (a.x, a.y),
            7,
            fill=STATE_COLORS["in_queue"],
            color=(0, 0, 0, 0),
            parent="agents",
        )