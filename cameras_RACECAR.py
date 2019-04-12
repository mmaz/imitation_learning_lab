class Video:
    # dev/video*
    LEFT = 1
    CENTER = 2
    RIGHT = 0

    @classmethod
    def notify(cls):
        print(":::: USING LEFT=/dev/video{}, CENTER=/dev/video{}, RIGHT=/dev/video{} ::::".format(cls.LEFT, cls.CENTER, cls.RIGHT))