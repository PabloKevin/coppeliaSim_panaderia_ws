-- Set spawn interval (call from another script)
function setSpawnInterval(interval)
    config.spawnInterval = interval
    print("Spawn interval set to: " .. interval .. "s")
end

-- Trigger immediate spawn
function triggerSpawn()
    spawnObject()
    lastSpawnTime = sim.getSimulationTime()
end

-- Enable/disable generator
function setGeneratorActive(state)
    config.active = state
    print("Generator active: " .. tostring(state))
end

-- Set linear spawn position offset
function setSpawnOffset(offset)
    config.spawnPosition = offset
    print("Spawn offset set to: " .. table.concat(offset, ", "))
end

-- Set exact spawn position
function setSpawnPosition(x, y, z)
    config.spawnPosition = {x, y, z}
    sim.setObjectPosition(selfHandle, -1, {x, y, z})
end