from classes.letters import *
from classes.base_letters import String, Letter, Stroke, get_2d_visualization

if __name__ == "__main__":
    cuhk = String(
        letters=[
            get_C(),
            get_U(),
            get_H(),
            get_K()
        ]
    )

    get_2d_visualization(cuhk, "cuhk.png")