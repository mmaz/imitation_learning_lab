class Video:
    # dev/video*
    LEFT = 0
    RIGHT = 1

    @classmethod
    def notify(cls):
        print(
            ":::: Camera IDs: LEFT=/dev/video{}, RIGHT=/dev/video{} ::::".format(
                cls.LEFT, cls.RIGHT
            )
        )

