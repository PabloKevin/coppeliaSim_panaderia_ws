-- object_generator.lua
-- Attach this script to a dummy object positioned above the conveyor

-- Configuration
config = {
    shapeFiles = {
        "/home/user/shapes/cube.stl",
        "/home/user/shapes/sphere.stl",
        "/home/user/shapes/cylinder.stl",
        "/home/user/shapes/custom_shape.stl"
    },
    spawnInterval = 3.0,  -- seconds between spawns
    conveyorSpeed = 0.1,  -- m/s
    spawnHeight = 0.15,   -- height above conveyor
    spawnPosition = {0, 0, 0.1},  -- relative to generator
    shapeScale = 0.001,   -- scale factor for imported shapes
    randomShapes = true,  -- randomly select shapes
    currentShapeIndex = 1 -- used when randomShapes = false
}

-- State variables
local lastSpawnTime = 0
local spawnedObjects = {}
local objectCounter = 0

function sysCall_init()
    -- Get generator handle
    selfHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    
    
    -- Initialize random seed
    math.randomseed(os.time())
    
    print("Object Generator Initialized")
    print("Spawn interval: " .. config.spawnInterval .. "s")
end

function sysCall_actuation()
    local currentTime = sim.getSimulationTime()
    
    -- Check if it's time to spawn new object
    if currentTime - lastSpawnTime >= config.spawnInterval then
        spawnObject()
        lastSpawnTime = currentTime
    end
    
    -- Clean up objects that have fallen off
    cleanupObjects()
end

function spawnObject()
    objectCounter = objectCounter + 1
    
    -- Select shape file
    local shapeFile
    if config.randomShapes then
        shapeFile = config.shapeFiles[math.random(#config.shapeFiles)]
    else
        shapeFile = config.shapeFiles[config.currentShapeIndex]
        config.currentShapeIndex = (config.currentShapeIndex % #config.shapeFiles) + 1
    end
    
    -- Import the shape
    local shapeHandle = sim.importShape(0, shapeFile, 0, 0, config.shapeScale, 1)
    
    if shapeHandle ~= -1 then
        -- Set object properties
        sim.setObjectAlias(shapeHandle, "SpawnedObject_"..objectCounter)
        
        -- Enable dynamic behavior
        sim.setShapeMassAndInertia(shapeHandle, 0.1, {0.01, 0.01, 0.01}, {0, 0, 0}, {1, 0, 0, 0})
        sim.setShapeDynamic(shapeHandle, true)
        
        -- Set initial position (above conveyor)
        local spawnPos = sim.getObjectPosition(selfHandle, -1)
        spawnPos[3] = spawnPos[3] + config.spawnHeight
        sim.setObjectPosition(shapeHandle, -1, spawnPos)
        
        -- Store reference
    table.insert(spawnedObjects, {
            handle = shapeHandle,
            spawnTime = sim.getSimulationTime(),
            position = spawnPos
        })
        
        print("Spawned object: " .. shapeHandle .. " from " .. shapeFile)
    else
        print("Failed to import shape: " .. shapeFile)
    end
end


function cleanupObjects()
    local currentTime = sim.getSimulationTime()
    local objectsToRemove = {}
    
    for i, obj in ipairs(spawnedObjects) do
        local pos = sim.getObjectPosition(obj.handle, -1)
        
        -- Remove if fallen off conveyor (z < -0.5) or too old (>30s)
        if pos[3] < -0.5 or (currentTime - obj.spawnTime) > 30 then
            sim.removeObject(obj.handle)
            table.insert(objectsToRemove, i)
        end
    end
    
    -- Remove from list (reverse order to maintain indices)
    for i = #objectsToRemove, 1, -1 do
        table.remove(spawnedObjects, objectsToRemove[i])
    end
end

function sysCall_cleanup()
    -- Clean up all spawned objects
    for _, obj in ipairs(spawnedObjects) do
        sim.removeObject(obj.handle)
    end
    spawnedObjects = {}
end