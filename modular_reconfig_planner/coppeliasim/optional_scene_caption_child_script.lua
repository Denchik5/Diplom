-- Optional child script for a dummy object named /SceneStatus.
-- The C++ planner does not require this script. It is intended only for visual debugging.

function sysCall_init()
    sim = require('sim')
    statusText = 'Modular reconfiguration demo ready'
    drawing = sim.addDrawingObject(sim.drawing_lines, 2, 0, -1, 200, {0.1, 0.6, 1.0})
end

function sysCall_sensing()
    -- Extend here if trajectory lines or text overlays are needed.
end

function setStatusText(text)
    statusText = text
    print('[SceneStatus] ' .. text)
end

function addTrajectorySegment(x1, y1, z1, x2, y2, z2)
    if drawing then
        sim.addDrawingObjectItem(drawing, {x1, y1, z1, x2, y2, z2})
    end
end
