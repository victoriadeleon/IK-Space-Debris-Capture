sim=require'sim'
simUI=require'simUI'

function sysCall_init()
    jointHandles={{-1,-1,-1},{-1,-1,-1},{-1,-1,-1}}
    firstPartTorqueSensorHandles={-1,-1,-1}
    for i=0,2,1 do
        if (i~=1) then
            jointHandles[i+1][1]=sim.getObject('../jointA_'..i)
        end
        jointHandles[i+1][2]=sim.getObject('../jointB_'..i)
        jointHandles[i+1][3]=sim.getObject('../jointC_'..i)
        firstPartTorqueSensorHandles[i+1]=sim.getObject('../jointB_'..i)
    end
    modelHandle=sim.getObject('..')
    closing=false
    sliderV=50
    firstPartLocked={false,false,false}
    needFullOpening={0,0,0}
    firstPartTorqueOvershootCount={0,0,0}
    firstPartTorqueOvershootCountRequired=1
    firstPartMaxTorque=0.9

    closingVel=60*math.pi/180
    openingVel=-120*math.pi/180
    closingOpeningTorque=1

    for i=1,3,1 do
        sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_motor_enabled,1)
        sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_ctrl_enabled,0)
        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_motor_enabled,1)
        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_ctrl_enabled,1)
        sim.setJointTargetForce(jointHandles[i][2],closingOpeningTorque)
        sim.setJointTargetForce(jointHandles[i][3],closingOpeningTorque)
        sim.setJointTargetVelocity(jointHandles[i][2],-closingVel)
        sim.setJointTargetVelocity(jointHandles[i][3],-closingVel/3)
    end
end

function sysCall_cleanup() 
    for i=1,3,1 do
        sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_motor_enabled,1)
        sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_ctrl_enabled,0)
        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_motor_enabled,1)
        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_ctrl_enabled,1)
        sim.setJointTargetForce(jointHandles[i][2],closingOpeningTorque)
        sim.setJointTargetForce(jointHandles[i][3],closingOpeningTorque)
        sim.setJointTargetVelocity(jointHandles[i][2],-closingVel)
        sim.setJointTargetVelocity(jointHandles[i][3],-closingVel/3)
    end
end 

function sysCall_sensing()
    local s=sim.getObjectSel()
    local show=(s and #s==1 and s[1]==modelHandle)
    if show then
        if not ui then
            local xml =[[<ui title="xxxx" closeable="false" placement="relative" layout="form">
                    <button id="1" text="open" checkable="true" checked="true" auto-exclusive="true" on-click="openClicked"/>
                    <button id="2" text="close" checkable="true" auto-exclusive="true" on-click="closeClicked"/>
                    
                    <label text="Finger angle"/>
                    <hslider id="3" on-change="fingerAngleMoved"/>
            </ui>]]
            ui=simUI.create(xml)
            if uiPos then
                simUI.setPosition(ui,uiPos[1],uiPos[2])
            else
                uiPos={}
                uiPos[1],uiPos[2]=simUI.getPosition(ui)
            end
            simUI.setTitle(ui,sim.getObjectAlias(modelHandle,5))
            simUI.setButtonPressed(ui,1,not closing)
            simUI.setButtonPressed(ui,2,closing)
            simUI.setSliderValue(ui,3,sliderV)
        end
    else
        if ui then
            uiPos[1],uiPos[2]=simUI.getPosition(ui)
            simUI.destroy(ui)
            ui=nil
        end
    end
end

function openClicked(ui,id)
    closing=false
end

function closeClicked(ui,id)
    closing=true
end

function fingerAngleMoved(ui,id,v)
    sliderV=v
    sim.setJointTargetPosition(jointHandles[1][1],-math.pi*0.5+math.pi*sliderV/100)
    sim.setJointTargetPosition(jointHandles[3][1],-math.pi*0.5+math.pi*sliderV/100)
end

function sysCall_actuation()
    local angleSignal = sim.getFloatSignal('Finger_Angle')
    if angleSignal then
        local angle = -math.pi*0.5 + math.pi * angleSignal / 100
        sim.setJointTargetPosition(jointHandles[1][1], angle)
        sim.setJointTargetPosition(jointHandles[3][1], angle)
        sim.clearFloatSignal('Finger_Angle') -- evita que la mano se cierre desde el inicio
    end

    for i=1,3,1 do
        if (closing)and(needFullOpening[1]~=2)and(needFullOpening[2]~=2)and(needFullOpening[3]~=2) then
            if (firstPartLocked[i]) then
                sim.setJointTargetVelocity(jointHandles[i][3],closingVel/3)
            else
                t=simJointGetForce(firstPartTorqueSensorHandles[i])
                if (t)and(t<-firstPartMaxTorque) then
                    firstPartTorqueOvershootCount[i]=firstPartTorqueOvershootCount[i]+1
                else
                    firstPartTorqueOvershootCount[i]=0
                end
                if (firstPartTorqueOvershootCount[i]>=firstPartTorqueOvershootCountRequired) then
                    needFullOpening[i]=1
                    firstPartLocked[i]=true
                    sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_ctrl_enabled,1)
                    sim.setJointTargetForce(jointHandles[i][2],closingOpeningTorque*100)
                    sim.setJointTargetPosition(jointHandles[i][2],sim.getJointPosition(jointHandles[i][2]))
                    sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_ctrl_enabled,0)
                    sim.setJointTargetVelocity(jointHandles[i][3],closingVel/3)
                else
                    sim.setJointTargetVelocity(jointHandles[i][2],closingVel)
                    sim.setJointTargetPosition(jointHandles[i][3],(45*math.pi/180)+sim.getJointPosition(jointHandles[i][2])/3)
                end
            end
        else
            if (needFullOpening[i]==1) then
                needFullOpening[i]=2
            end
            sim.setJointTargetVelocity(jointHandles[i][3],openingVel/3)
            if (firstPartLocked[i]) then
                jv=sim.getJointPosition(jointHandles[i][3])
                if (jv<45.5*math.pi/180) then
                    firstPartLocked[i]=false
                    sim.setObjectInt32Param(jointHandles[i][2],sim.jointintparam_ctrl_enabled,0)
                    sim.setJointTargetForce(jointHandles[i][2],closingOpeningTorque)
                    sim.setJointTargetVelocity(jointHandles[i][2],openingVel)
                end
            else
                if (needFullOpening[i]~=0) then
                    jv3=sim.getJointPosition(jointHandles[i][3])
                    jv2=sim.getJointPosition(jointHandles[i][2])
                    if (jv3<45.5*math.pi/180)and(jv2<2*math.pi/180) then
                        needFullOpening[i]=0
                        sim.setObjectInt32Param(jointHandles[i][3],sim.jointintparam_ctrl_enabled,1)
                        sim.setJointTargetPosition(jointHandles[i][3],(45*math.pi/180)+sim.getJointPosition(jointHandles[i][2])/3)
                    end
                else
                    sim.setJointTargetVelocity(jointHandles[i][2],openingVel)
                    sim.setJointTargetPosition(jointHandles[i][3],(45*math.pi/180)+sim.getJointPosition(jointHandles[i][2])/3)
                end
            end
        end
    end
end
