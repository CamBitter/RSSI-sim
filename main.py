from gui import setup_ui, redraw
from sim import Sim
from agent import Agent
import dearpygui.dearpygui as dpg


def main():

    W, H = 1280, 800

    sim_params = {
        "move_speed": 2,
        "proximity_threshold": 50,
        "wait_time": 300,
        "end_point": (W / 2, H / 10),
    }
   
    sim = Sim([], sim_params)
    setup_ui(W, H, sim)

    while dpg.is_dearpygui_running():
        sim.step()
        redraw(sim.agents, sim.end)
        dpg.render_dearpygui_frame()

    dpg.destroy_context()

if __name__ == "__main__":
    main()
