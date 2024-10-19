class Waypoint:
    def __init__(self, lat, lon, alt, hdg):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.hdg = hdg

    def __hash__(self):
        return hash(self.lat) ^ hash(self.lon) ^ hash(self.alt) ^ hash(self.hdg)
