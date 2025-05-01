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
                    [70, 90, 0],
                    [50, 95, 0],
                    [30, 80, 0],
                    [20, 50, 0],
                    [30, 20, 0],
                    [50, 5, 0],
                    [70, 10, 0],
                    [80, 20, 0]
                ],
                time_list=[0, 1.5, 3, 4.5, 6, 7.5, 9, 10.5, 12]
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
                    [20, 95, 0],
                    [20, 30, 0],
                    [50, 5, 0],
                    [80, 30, 0],
                    [80, 95, 0]
                ],
                time_list=[0, 4, 7, 10, 14]
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
                time_list=[0, 5]
            ),
            Stroke(
                poses=[
                    [20, 50, 0],
                    [80, 50, 0],
                ],
                time_list=[0, 4]
            ),
            Stroke(
                poses=[
                    [80, 95, 0],
                    [80, 5, 0],
                ],
                time_list=[0, 5]
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
                time_list=[0, 5]
            ),
            Stroke(
                poses=[
                    [75, 95, 0],
                    [20, 40, 0]
                ],
                time_list=[0, 4]
            ),
            Stroke(
                poses=[
                    [40, 60, 0],
                    [75, 5, 0]
                ],
                time_list=[0, 4]
            )
        ]
    )