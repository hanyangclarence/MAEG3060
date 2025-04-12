from .base_letters import Stroke, Letter

# TODO: These parts need adjustment to make the letters more realistic

def get_C() -> Letter:
    """
    Get the letter C.
    """
    return Letter(
        letter="C",
        strokes=[
            Stroke(
                poses=[
                    [80, 80, 0],
                    [75, 85, 0],
                    [25, 85, 0],
                    [25, 15, 0],
                    [75, 15, 0],
                    [80, 20, 0]
                ],
                time_list=[0, 0.1, 0.4, 0.7, 1.0, 1.1]
            )
        ]
    )


def get_U() -> Letter:
    """
    Get the letter U.
    """
    return Letter(
        letter="U",
        strokes=[
            Stroke(
                poses=[
                    [80, 95, 0],
                    [80, 30, 0],
                    [50, 5, 0],
                    [20, 30, 0],
                    [20, 95, 0]
                ],
                time_list=[0, 0.3, 0.6, 0.9, 1.2]
            )
        ]
    )


def get_H() -> Letter:
    """
    Get the letter H.
    """
    return Letter(
        letter="H",
        strokes=[
            Stroke(
                poses=[
                    [20, 95, 0],
                    [20, 5, 0],
                ],
                time_list=[0, 0.3]
            ),
            Stroke(
                poses=[
                    [20, 50, 0],
                    [80, 50, 0],
                ],
                time_list=[0, 0.3]
            ),
            Stroke(
                poses=[
                    [80, 95, 0],
                    [80, 5, 0],
                ],
                time_list=[0, 0.3]
            ),
        ]
    )

def get_K() -> Letter:
    return Letter(
        letter="K",
        strokes=[
            Stroke(
                poses=[
                    [20, 95, 0],
                    [20, 5, 0],
                ],
                time_list=[0, 0.3]
            ),
            Stroke(
                poses=[
                    [75, 95, 0],
                    [20, 40, 0]
                ],
                time_list=[0, 0.3]
            ),
            Stroke(
                poses=[
                    [40, 55, 0],
                    [75, 5, 0]
                ],
                time_list=[0, 0.3]
            )
        ]
    )