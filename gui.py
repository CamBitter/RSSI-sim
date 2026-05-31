import dearpygui.dearpygui as dpg

STATE_COLORS = {
    "in_queue": (255, 200, 60, 255),
    "queue_end": (255, 120, 60, 255),
}

def setup_ui(W, H, sim):

    dpg.create_context()
    dpg.create_viewport(title="Sim", width=W, height=H)

    # Setup canvas window
    with dpg.window(label="Sim", no_scrollbar=True):
        with dpg.drawlist(width=W, height=H, tag="canvas"):
            dpg.add_draw_layer(tag="floor")  
            dpg.add_draw_layer(tag="agents")  

    # Draw floor
    dpg.draw_rectangle((0, 0), (W, H), fill=(28, 32, 38, 255), parent="floor")
    dpg.draw_circle(sim.end, 7, fill=STATE_COLORS["queue_end"], color=(0,0,0,0), parent="floor")

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