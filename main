sim = require 'sim'
simIK = require 'simIK'

local reachedReported = false
local isAdjusting = false
local targetZ = 0.7
local satellite, proxSensor, attachPoint

local z_est = 0.7
local z_err = 1.0
local z_meas_err = 0.05
local prev_z = nil
local v_est = 0.0

function sysCall_init()
    self = sim.getObject('.')
    simBase = sim.getObject('/UR5')
    simTip = sim.getObject(':/BarrettHand/tip')
    simTarget = sim.getObject(':/target')

    satellite = sim.getObject('/satellite')
    proxSensor = sim.getObject(':/BarrettHand/tip/Proximity_sensor')
    attachPoint = sim.getObject(':/BarrettHand/attachPoint')

    local satPos = sim.getObjectPosition(satellite, -1)
    z_est = satPos[3]
    prev_z = z_est

    ikEnv = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv, ikGroup, simIK.method_damped_least_squares, 0.1, 10)
    simIK.addElementFromScene(ikEnv, ikGroup, simBase, simTip, simTarget, simIK.constraint_pose)

    local basePos = sim.getObjectPosition(simBase, -1)
    print(string.format("[INFO] UR5 base position: %.3f %.3f %.3f", basePos[1], basePos[2], basePos[3]))
end

function sysCall_actuation()
    -- kalman prediction
    local satPos = sim.getObjectPosition(satellite, -1)
    local z_meas = satPos[3]

    -- kalman update
    local kg = z_err / (z_err + z_meas_err)
    z_est = z_est + kg * (z_meas - z_est)
    z_err = (1 - kg) * z_err

    -- velocity estimation
    local dt = sim.getSimulationTimeStep()
    v_est = (z_est - prev_z) / dt
    prev_z = z_est

    -- time prediction in regard to capture zone
    local z_objetivo = 0.45
    local t_estimado = (z_est - z_objetivo) / v_est

    print(string.format("[DEBUG] Z=%.3f | Vz=%.3f | T=%.2f", z_est, v_est, t_estimado or -1))

    -- triggered when conditions are met
    if v_est < -0.01 and t_estimado and t_estimado > 0 and t_estimado < 5 then
        print(string.format("[PREDICCION] Z estimada: %.3f m | Vel: %.3f m/s | T captura: %.2f s", z_est, v_est, t_estimado))
    end

    local result, distance, detectedPoint, detectedHandle = sim.checkProximitySensor(proxSensor, satellite)

    if result == 1 and detectedHandle == satellite then
        print("Satelite captured!")
        sim.setObjectParent(satellite, attachPoint, true)
        sim.setObjectPosition(satellite, attachPoint, {0, 0, 0.1})
        sim.setObjectOrientation(satellite, attachPoint, {-0.7, 0, 0.2})
        sim.setFloatSignal('Finger_Angle', 0)
    end

    local currentPos = sim.getObjectPosition(simTarget, -1)
    local currentZ = currentPos[3]
    local stepZ = 0.02

    if not isAdjusting then
        local zona = sim.getIntegerSignal("Vision_ZonaVertical")
        if zona == 0 and currentZ > 0.6 then
            targetZ = math.max(0.6, currentZ - stepZ)
            isAdjusting = true
        elseif zona == 2 and currentZ < 0.8 then
            targetZ = math.min(0.8, currentZ + stepZ)
            isAdjusting = true
        elseif zona == 1 then
            targetZ = currentZ
        end
    end

    local targetFinal = {-0.26805, 0, targetZ}
    local speed = 0.01
    local dx = targetFinal[1] - currentPos[1]
    local dy = targetFinal[2] - currentPos[2]
    local dz = targetFinal[3] - currentPos[3]
    local dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    local step = speed * sim.getSimulationTimeStep()

    local nextPos = {currentPos[1], currentPos[2], currentPos[3]}
    if dist > step then
        local ratio = step / dist
        nextPos[1] = currentPos[1] + dx * ratio
        nextPos[2] = currentPos[2] + dy * ratio
        nextPos[3] = currentPos[3] + dz * ratio
    else
        nextPos = targetFinal
        isAdjusting = false
    end

    if isWithinWorkspace(nextPos) then
        sim.setObjectPosition(simTarget, -1, nextPos)
        simIK.handleGroup(ikEnv, ikGroup, {syncWorlds=true, allowError=true})

        if not reachedReported then
            local error = getPositionalError(simTip, simTarget)
            if error <= 0.02 then
                print(string.format("[?] Tip reached target with error = %.3f m", error))
            else
                print(string.format("[x] Tip didn't reach target. Error = %.3f m", error))
            end
            reachedReported = true
        end
    else
        local d = distanceToBase(nextPos)
        print(string.format("[!] Target outside workspace. Distance = %.3f m (limit = 0.85 m)", d))
    end
end

function isWithinWorkspace(targetPos)
    return distanceToBase(targetPos) <= 0.85
end

function distanceToBase(pos)
    local basePos = sim.getObjectPosition(simBase, -1)
    local dx = pos[1] - basePos[1]
    local dy = pos[2] - basePos[2]
    local dz = pos[3] - basePos[3]
    return math.sqrt(dx*dx + dy*dy + dz*dz)
end

function getPositionalError(tipHandle, targetHandle)
    local tipPos = sim.getObjectPosition(tipHandle, -1)
    local targetPos = sim.getObjectPosition(targetHandle, -1)
    local dx = tipPos[1] - targetPos[1]
    local dy = tipPos[2] - targetPos[2]
    local dz = tipPos[3] - targetPos[3]
    return math.sqrt(dx*dx + dy*dy + dz*dz)
end
