class WallFollower:
    def __init__(self, speed=0.3, distance_threshold=0.4, side="right"):
        self.speed = speed
        self.distance_threshold = distance_threshold
        self.side = side

    def follow_wall(self, scan_data):
        if self.side == "right":
            front_distance = scan_data[0]
            side_distance = scan_data[90]
        else:
            front_distance = scan_data[180]
            side_distance = scan_data[90]

        error = side_distance - self.distance_threshold
        return min(self.speed, max(0.0, self.speed * error))

if __name__ == '__main__':
    wall_follower = WallFollower()
    # Integration with other components can be added here if needed
