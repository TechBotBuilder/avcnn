class MiniZone:
    def __init__(self, minx, miny, maxx, maxy):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
    def contains_position(self, positionx, positiony):
        return (self.minx <= positionx <= self.maxx) and (self.miny <= positiony <= self.maxy)

class Zone:
    def __init__(self, initzone, middlezone, endzone, completion_reward, maxcompletions=1):
        self.initzone = initzone
        self.middlezone = middlezone
        self.endzone = endzone
        self.completion_reward = completion_reward
        self.maxcompletions = maxcompletions
        self.hasinit = False
        self.hasmiddle = False
        self.hasend = 0
    def update(self, positionx, positiony):
        ininitzone = self.initzone.contains_position(positionx, positiony)
        inmiddlezone = self.middlezone.contains_position(positionx, positiony)
        inendzone = self.endzone.contains_position(positionx, positiony)
        if ( (not ininitzone) and (not inmiddlezone) and (not inendzone) ):
            self.hasinit = False
            self.hasmiddle = False
            return 0
        if ininitzone:
            self.hasinit = True
        if inmiddlezone and self.hasinit:
            self.hasmiddle = True
        if inendzone and self.hasmiddle and (self.hasend < self.maxcompletions):
            self.hasend += 1
            self.hasinit = False
            self.hasmiddle = False
            return self.completion_reward
        return 0
    def reset(self):
        self.hasinit = False
        self.hasmiddle = False
        self.hasend = 0

class RewardWatcher:
    def __init__(self, rewarder):
        self.positionx = 0
        self.positiony = 0
        self.cornersdone = [0,0,0,0]
        self.zones = [
            Zone(MiniZone(87.969,233.0,202.0,280.5),#corner Northeast DONE
                MiniZone(87.969,280.47,202.0,367.0),
                MiniZone(40.0,280.47,88.0,367.0), 25.0),
            Zone(MiniZone(-101.0,278.7,-58.0,370.0),#corner Northwest DONE
                MiniZone(-150.0,278.7,-100.71,370.0),
                MiniZone(-150.0,235.0,-100.71,279.1), 25.0),
            Zone(MiniZone(-190.0,-54.8,-100.46,-17.0),#corner Southwest DONE
                MiniZone(-190.0,-91.0,-100.46,-54.27),
                MiniZone(-100.8,-91.0,-59.0,-54.27), 25.0),
            Zone(MiniZone(40.0,-90.0,90.5,-53.08),#corner Southeast DONE
                MiniZone(90.0,-90.0,130.0,-53.08),
                MiniZone(90.0,-53.6,130.0,-16.6), 25.0),
            Zone(MiniZone(124.8,171.0,143.0,188.37),#hoop DONE
                #MiniZone(124.8,188.32,143.0,189.36),
                MiniZone(124.8,187.32,143.0,190.36),
                MiniZone(128.4,189.31,143.0,206.68), 50.0),
            Zone(MiniZone(-166.8,139.1,-133.0,152.0),#ramp DONE
                MiniZone(-157.94,124.8,-143.88,139.6),
                MiniZone(-166.8,112.0,-133.0,125.04), 50.0),
            Zone(MiniZone(89.45,25.0,180.26,51.0),#completion/goal DONE
                MiniZone(89.45,50.0,180.26,60.0),
                MiniZone(89.45,59.06,180.26,95.55), 300.0)]
        self.rewarder = rewarder
    def update(self, data):
        reward = 0
        for zone in range(len(self.zones)):
            r = self.zones[zone].update(data['x'], data['y'])
            if (r > 0) and (0 <= zone <= 3): #corner
                goodcorner = ( sum(self.cornersdone) == zone )
                self.cornersdone[zone] = int(goodcorner)
                if not goodcorner:
                    r = 0
            if (r > 0) and (zone == len(self.zones)-1): #GOAL
                if sum(self.cornersdone) == 4:
                   rewarder.won = True
                else:
                    r = 0
            reward += r
        self.rewarder.addreward(reward)
    def reset(self):
        for zone in self.zones:
            zone.reset()
        self.positionx = 0
        self.positiony = 0
        self.cornersdone = [0,0,0,0]
