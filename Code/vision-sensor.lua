function sysCall_init()
    sensorH = sim.getObject('..')
    local res = sim.getVisionSensorRes(sensorH)
    resX = res[1]
    resY = res[2]
    
    -- buffer between zones (5%)
    local buffer = math.floor(resY * 0.03)
    
    limite_inferior = math.floor(resY/3) + buffer  
    limite_superior = math.floor(2*resY/3) - buffer 
    
    sim.addLog(sim.verbosity_scriptinfos, string.format("Sensor %d: Zonas [0-%d]=ABAJO | [%d-%d]=MEDIO | [%d-%d]=ARRIBA", 
                     sensorH, limite_inferior, limite_inferior+1, limite_superior-1, limite_superior, resY))
end

function sysCall_vision(inData)
    local img = sim.getVisionSensorImg(inData.handle)
    
    if img then
        local zonas = {arriba = 0, medio = 0, abajo = 0}
        local step = 20  -- sampling every 20 pixels
        local threshold = 100  -- to avoid noise
        
        for i = 1, #img, 3 * step do
            local r = string.byte(img, i)
            local g = string.byte(img, i+1)
            local b = string.byte(img, i+2)
            
            local luminancia = 0.299*r + 0.587*g + 0.114*b
            if luminancia > threshold then
                local pixel_num = math.floor((i-1)/3)
                local y = math.floor(pixel_num / resX) 

                if y <= limite_inferior then
                    zonas.abajo = zonas.abajo + 1
                elseif y >= limite_superior then
                    zonas.arriba = zonas.arriba + 1
                else
                    zonas.medio = zonas.medio + 1
                end
            end
        end

        local max_zona = "medio"
        local umbral_dominancia = 1.2 
        
        if zonas.arriba > (zonas.medio * umbral_dominancia) and 
           zonas.arriba > (zonas.abajo * umbral_dominancia) then
            max_zona = "arriba"
        elseif zonas.abajo > (zonas.medio * umbral_dominancia) and 
               zonas.abajo > (zonas.arriba * umbral_dominancia) then
            max_zona = "abajo"
        end

        local signalValue = ({arriba=2, medio=1, abajo=0})[max_zona]
        sim.setIntegerSignal("Vision_ZonaVertical", signalValue)
    end
    return {trigger=true}
end
