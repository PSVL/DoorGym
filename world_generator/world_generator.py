#!/usr/bin/python3
from mjcf import elements as e
from mjcf.elements import visual as v
from random import randrange
from tqdm import tqdm
import argparse
import os
import json


def main(dataset_file):
    #Path maker
    handle_folder = os.path.join(args.input_dirname.format(knob_type), dataset_file)
    stl_path = os.path.join('door/{}knobs'.format(knob_type), dataset_file)
    with open(os.path.join(handle_folder, 'info.json'), 'r') as f:
        params_dict = json.load(f)

    #geom parts number
    door_parts_n = 1
    frame_parts_n = 6
    wall_parts_n = 3
    knob_parts_n = params_dict['model_count']

    if args.robot_type == "gripper":
        robot_type = "blue_gripper"
    elif args.robot_type == "hook":
        robot_type = "blue_hook"
    elif args.robot_type == "floatinggripper":
        robot_type = "blue_floatinggripper"
    elif args.robot_type == "floatinghook":
        robot_type = "blue_floatinghook"
    elif args.robot_type == "mobile_gripper":
        robot_type = "blue_mobile_gripper"
    elif args.robot_type == "mobile_hook":
        robot_type = "blue_mobile_hook"
    else:
        raise Exception("robot type not recognized")
    
    ###################### Randomize Parameters ######################
    # Lighting property
    light_n=randrange(2,6)
    light_property_list = []
    for i in range(light_n):
        light_property = dict(
            light_diffuse=[randrange(9,11)/10, randrange(9,11)/10, randrange(9,11)/10],
            light_pos=[randrange(0,500)/100.0, randrange(-500,500)/100.0, randrange(300,700)/100.0],
            light_dir=[randrange(-50,50)/100.0, randrange(-50,50)/100.0, randrange(-50,-25)/100.0]
        )
        light_property_list.append(light_property)

    # Camera property
    camera1_pos = [0.99, -0.5, 1.0] #1m x 1m
    camera1_ori = [0.0, 1.57, 1.57]
    camera2_pos = [0.5, 0.0, 1.99]  #1m x 1m
    camera2_ori = [0, 0, 0]
    camera_fieldview = 60
    camera_randomization = False
    if camera_randomization:
        for i in range(len(camera1_pos)):
            camera1_pos[i] += randrange(-15,15)/1000
            camera2_pos[i] += randrange(-15,15)/1000
        for i in range(len(camera1_ori)):
            camera1_ori[i] += randrange(-17,17)/1000
            camera2_ori[i] += randrange(-17,17)/1000
        camera_fieldview_noise = [randrange(-100,100)/100, randrange(-100,100)/100]
    camera_poses = [camera1_pos, camera2_pos]
    camera_ories = [camera1_ori, camera2_ori]
    camera_fieldview_noise = [0, 0]

    #Material list
    material_name_list = ['Paint','Wood','Carpet','Metal']
    paint_shininess = randrange(1,50)/100.0
    paint_specular = randrange(1,50)/100.0
    wood_shininess = randrange(1,20)/100.0
    wood_specular = randrange(1,20)/100.0
    carpet_shininess = randrange(1,5)/100.0
    carpet_specular = randrange(1,5)/100.0
    metal_shininess = randrange(80,100)/100.0
    metal_specular = randrange(80,100)/100.0

    # Wall property
    wall_rgba = [randrange(1,85)/100.0, randrange(1,85)/100.0, randrange(1,85)/100.0, 1.0]
    wall_location = [0.0, randrange(-100,100)/1000.0, 0.0]
    wall_material = material_name_list[randrange(0,3)]

    # Frame property
    frame_rgba = [randrange(70,85)/100.0, randrange(70,85)/100.0, randrange(70,85)/100.0, 1.0]
    frame_material = material_name_list[randrange(0,3)]

    # Door Frame Joint Property  
    door_frame_damper = randrange(10, 20)/100.0
    door_frame_spring = randrange(10, 20)/100.0
    door_frame_frictionloss = randrange(0,1)

    # Door hinge direction
    if args.righthinge_ratio > randrange(0,100)/100.0:
        hinge_loc = "righthinge"
    else:
        hinge_loc = "lefthinge"
    
    # Opening direction
    if args.pulldoor_ratio > randrange(0,100)/100.0:
        opendir = "pull"
    else:
        opendir = "push"

    # Door property
    door_height = randrange(2000, 2500)/1000.0
    door_width = randrange(800, 1200)/1000.0
    door_thickness = randrange(20, 30)/1000.0
    knob_height = randrange(990, 1010)/1000.0 # legal height of the door knob is 864mm to 1219mm
    knob_horizontal_location_ratio = randrange(10,20)/100.0 #height-ratio and from-side-ratio of knob
    door_mass = door_height*door_width*door_thickness*randrange(200,300) # Density of MDF is in range of 500-1000kg/m^3 
    door_rgba = [randrange(1,100)/100.0, randrange(1,100)/100.0, randrange(1,100)/100.0, 1.0]
    door_material = material_name_list[randrange(0,3)]

    # Knob Door Joint Property
    knob_door_damper = randrange(10, 20)/100.0
    knob_door_spring = randrange(10, 15)/100.0
    knob_door_frictionloss = randrange(0, 1)
    knob_rot_range = randrange(75, 80)*3.14/180

    # Knob property
    if hinge_loc == "righthinge":
        doorknob_pos = [0,0,0]
        knob_euler = [-1.57,1.57,0]
    elif hinge_loc == "lefthinge":
        doorknob_pos = [0, (0.5-knob_horizontal_location_ratio)*door_width*2, 0]
        knob_euler = [1.57,1.57,0]
    else:
        raise Exception("door direction undefinded")

    knob_mass = randrange(1,2)
    knob_rgba = [randrange(1,100)/100.0, randrange(1,100)/100.0, randrange(1,100)/100.0, 1.0]
    knob_surface_friction = [randrange(50,100)/100.0, randrange(1,5)/1000.0, randrange(1,5)/1000.0]
    doorknob_material = material_name_list[randrange(0,3)]


    high_variance = False
    if high_variance:
        door_frame_damper = randrange(20, 60)/100.0
        door_frame_spring = randrange(20, 60)/100.0
        door_mass = door_height*door_width*door_thickness*randrange(200,600) # (700,850) Density of MDF
        knob_surface_friction = [randrange(0,100)/100.0, randrange(1,10)/1000.0, randrange(1,10)/1000.0]

    knob_color_overfit = False
    if knob_color_overfit:
        material_name_list = ['Paint','Wood','Carpet','Metal']
        wall_rgba = [randrange(90,95)/100.0, randrange(90,95)/100.0, randrange(90,95)/100.0, 1.0]
        wall_material = material_name_list[randrange(0,2)]
        frame_rgba = [randrange(90,100)/100.0, randrange(80,100)/100.0, randrange(80,100)/100.0, 1.0]
        frame_material = material_name_list[randrange(0,2)]
        door_rgba = [randrange(90,95)/100.0, randrange(90,95)/100.0, randrange(90,95)/100.0, 1.0]
        door_material = material_name_list[randrange(0,2)]
        knob_color = randrange(0,100)
        if knob_color < 34:
            # Brass
            knob_rgba = [randrange(71,72)/100.0, randrange(66,67)/100.0, randrange(25,26)/100.0, 1.0]
        elif 34 <= knob_color < 67:
            # Silver
            knob_rgba = [randrange(95,100)/100.0, randrange(95,100)/100.0, randrange(95,100)/100.0, 1.0]
        else:
            # Black
            knob_rgba = [randrange(1,10)/100.0, randrange(1,10)/100.0, randrange(1,10)/100.0, 1.0]
        doorknob_material = material_name_list[3]

        light_n=randrange(6,8)
        light_property_list = []
        for i in range(light_n):
            light_property = dict(
                light_diffuse=[randrange(9,11)/10, randrange(9,11)/10, randrange(9,11)/10],
                light_pos=[randrange(0,500)/100.0, randrange(-500,500)/100.0, randrange(300,700)/100.0],
                light_dir=[randrange(-50,50)/100.0, randrange(-50,50)/100.0, randrange(-100,-80)/100.0]
            )
            light_property_list.append(light_property)

    # Robot property
    robot_damping = randrange(9, 11)/100

    ###################### Constant Parameters ######################
    # gravity
    if args.robot_type.find("floating")>-1:
        gravity_vector = [0,0,0]
    else:
        gravity_vector = [0,0,-9.81]

    # Stationaly Camera
    camera_n = 2

    # World Wall Joint Property
    wall_world_armature = 0.0001
    wall_world_damper = 100000000
    wall_world_spring = 1000
    wall_world_frictionloss = 0
    wall_world_joint_pos = [0,0,0]


    # wall Frame Joint Property
    frame_wall_armature = 0.0001
    frame_wall_damper = 100000000
    frame_wall_spring = 1000
    frame_wall_frictionloss = 0
    frame_wall_joint_pos = [0,0,0]

    # Door stopper property
    doorstopper_size = [door_thickness/2.0, 0.05, 0.025]
    if opendir == "push":
        if hinge_loc == "righthinge":
            doorstopper_pos = [door_thickness*2.3, -door_width*knob_horizontal_location_ratio+0.15, door_height-knob_height-0.05]
        else:
            doorstopper_pos = [door_thickness*2.3, door_width-door_width*knob_horizontal_location_ratio-0.15, door_height-knob_height-0.05]
    else:
        if hinge_loc == "righthinge":
            doorstopper_pos = [0-door_thickness*0.7, -door_width*knob_horizontal_location_ratio+0.15, door_height-knob_height-0.05]
        else:
            doorstopper_pos = [0-door_thickness*0.7, door_width-door_width*knob_horizontal_location_ratio-0.15, door_height-knob_height-0.05]

    # Door Frame Joint Property
    door_frame_armature = 0.0001
    door_frame_limited = False
    door_frame_range = [-0.5, 0.5]
    if hinge_loc == "righthinge":
        door_frame_joint_pos = [0,door_width-door_width*knob_horizontal_location_ratio,0]
    else:
        door_frame_joint_pos = [0,-door_width*knob_horizontal_location_ratio,0]
    
    # Door property
    door_diaginertia = [door_mass/12.0*(door_height**2+door_width**2),
                        door_mass/12.0*(door_height**2+door_thickness**2),
                        door_mass/12.0*(door_width**2+door_thickness**2)]
    door_euler = [0,0,0]
    door_pos = [door_thickness*1.1,0,0]

    # Knob Door Joint Property
    knob_door_joint_pos = [0,0,0]
    knob_door_armature = 0.0001
    knob_door_limited = True
    knob_door_range = [-knob_rot_range, knob_rot_range]
    latch_thickness = 0.025
    latch_height = 0.05
    latch_width = (door_width * knob_horizontal_location_ratio) + 0.04
    latch_gap = latch_thickness*1.05

    # Knob property
    if knob_type == "pull":
        knob_pos = [door_thickness/2-0.006,0,0]
    else:
        knob_pos = [door_thickness/2,0,0]
    knob_diaginertia = [knob_mass/12.0*(latch_height**2+latch_width**2),
                        knob_mass/12.0*(latch_height**2+latch_thickness**2),
                        knob_mass/12.0*(latch_width**2+latch_thickness**2)]

    # Frame property
    frame_width = 100/1000.0
    overlap_door_frame = 10/1000.0
    frame_world_pos = [0, -(0.5-knob_horizontal_location_ratio)*door_width, knob_height]
    frame_euler = [0,0,0]
    frame_mass = 500
    frame_diaginertia = [0.0001, 0.0001, 0.0001]
    frame_pos_list = [
        [0, -door_width*knob_horizontal_location_ratio-frame_width/2.0-overlap_door_frame, (door_height+frame_width)/2.0-knob_height],
        [0, door_width-door_width*knob_horizontal_location_ratio+frame_width/2.0+overlap_door_frame, (door_height+frame_width)/2.0-knob_height],
        [0, door_width/2-door_width*knob_horizontal_location_ratio, door_height-knob_height+frame_width/2],
        [-(door_thickness+latch_gap), -door_width*knob_horizontal_location_ratio-frame_width/2.0-overlap_door_frame, (door_height+frame_width)/2.0-knob_height],
        [-(door_thickness+latch_gap), door_width-door_width*knob_horizontal_location_ratio+frame_width/2.0+overlap_door_frame, (door_height+frame_width)/2.0-knob_height],
        [-(door_thickness+latch_gap), door_width/2-door_width*knob_horizontal_location_ratio, door_height-knob_height+frame_width/2]]
    frame_size_list = [
        [door_thickness/2.0, frame_width/2.0, (door_height+frame_width)/2.0],
        [door_thickness/2.0, frame_width/2.0, (door_height+frame_width)/2.0],
        [door_thickness/2.0, door_width/2.0+frame_width, frame_width/2.0],
        [door_thickness/2.0, frame_width/2.0, (door_height+frame_width)/2.0],
        [door_thickness/2.0, frame_width/2.0, (door_height+frame_width)/2.0],
        [door_thickness/2.0, door_width/2.0+frame_width, frame_width/2.0]]

    # Wall property
    sidewall_len = 2000/1000.0
    topwall_width = 1000/1000.0
    wall_thickness = 300/1000.0
    wall_euler = [0,0,0]
    wall_mass = 100
    wall_diaginertia = [0.0001, 0.0001, 0.0001]
    wall_pos_list = [
        [0, -door_width/2.0-frame_width-sidewall_len/2.0+0.03, (door_height+frame_width)/2.0],
        [0, door_width/2.0+frame_width+sidewall_len/2.0, (door_height+frame_width)/2.0],
        [0, 0, door_height+frame_width+topwall_width/2.0]]
    wall_size_list = [
        [wall_thickness/2.0, sidewall_len/2.0, (door_height+frame_width)/2.0],
        [wall_thickness/2.0, sidewall_len/2.0, (door_height+frame_width)/2.0],
        [wall_thickness/2.0, door_width/2.0+frame_width+sidewall_len, topwall_width/2.0]]

    ###################### XML Generator ######################
    # Level 1
    mujoco = e.Mujoco(model="door_knob")

    #########################
    # Level 2
    compiler = e.Compiler(angle="radian")
    include = e.Include(file="../../robot/{}.xml".format(robot_type))
    option = e.Option(gravity=gravity_vector, timestep=0.001)
    visual = e.Visual()
    asset = e.Asset()
    contact = e.Contact()
    default = e.Default()
    worldbody = e.Worldbody()

    mujoco.add_children([
        compiler,
        include,
        option,
        visual,
        asset,
        contact,
        default,
        worldbody
    ])

    ######################
    # Level 3

    # Visual
    visual_list = []
    v_map = e.visual.Map(
        fogstart=3,
        fogend=5,
        force=0.1,
        znear=0.01,
        zfar=10
    )
    v_quality = v.Quality(
        shadowsize=2048
    )
    v_global = v.Global(
        offwidth=256,
        offheight=256
    )  
    visual_list.append(v_map)
    visual_list.append(v_quality)
    visual_list.append(v_global)
    visual.add_children(visual_list)

    # Asset
    if knob_type == "lever":
        knob_scale = [0.0015, 0.0015, 0.0015]
    else:
        knob_scale = [0.001, 0.001, 0.001]
    mesh_knob = []
    fname = '../../{0}/body_{1}.stl'
    name = 'door_knob_{}'
    for i in range(1,knob_parts_n+1):
        mesh_knob.append(e.Mesh(
            file=fname.format(stl_path,i),
            name=name.format(i),
            scale=knob_scale))

    texture_list = []
    texture_name_list = ['wall_geom','frame_geom','door_geom','knob_geom']
    texture_type_list = ['2d', '2d','2d','2d']
    for i in range(len(texture_name_list)):
        texture_list.append(e.Texture(
            builtin="flat",
            name=texture_name_list[i],
            height=32,
            width=32,
            type=texture_type_list[i]
        ))

    #Sky color
    texture_list.append(e.Texture(
        type="skybox",
        builtin="gradient",
        width=128,
        height=128,
        rgb1=[.4,.6,.8],
        rgb2=[0,0,0]
    ))

    #Floor color
    texture_list.append(e.Texture(
        name='texplane',
        type='2d',
        builtin='checker',
        rgb1=[0.2,0.3,0.4],
        rgb2=[0.1,0.15,0.2],
        width=512,
        height=512
    ))

    material_list = []
    material_shininess_list = [paint_shininess, wood_shininess, carpet_shininess, metal_shininess]
    material_specular_list = [paint_specular, wood_specular, carpet_specular, metal_specular]
    for i in range(len(material_name_list)):
        material_list.append(e.Material(
            name=material_name_list[i],
            texture=texture_name_list[i],
            shininess=material_shininess_list[i],
            specular=material_specular_list[i]
        ))

    material_list.append(e.Material(
            name="Floor",
            texture="texplane"
        ))

    asset_list = mesh_knob
    asset_list.extend(texture_list)
    asset_list.extend(material_list)
    asset.add_children(asset_list)


    #Contact
    doorstopper_contact = e.Pair(
        geom1="doorstopper",
        geom2="door0",
        solref="0.01 1"
    )
    latch_contacts = []
    for i in range(4):
        latch_contacts.append(e.Pair(
            geom1="knob_latch",
            geom2="door_frame_{}".format(i),
            solref="0.01 1"
        ))


    contact_list = [doorstopper_contact]
    if knob_type != "pull":
        contact_list.extend(latch_contacts)
    contact.add_children(contact_list)

    joint_default = e.Joint(
        armature=1,
        damping=1,
        limited="true")

    wall_default = e.Default(class_='wall')
    wall_geom = e.Geom(type='mesh', rgba=wall_rgba)
    wall_default.add_child(wall_geom)

    frame_default = e.Default(class_='frame')
    frame_geom = e.Geom(type='mesh', rgba=frame_rgba)
    frame_default.add_child(frame_geom)

    door_default = e.Default(class_='door')
    door_geom = e.Geom(type='mesh', rgba=door_rgba)
    door_default.add_child(door_geom)

    knob_default = e.Default(class_='door_knob')
    knob_geom = e.Geom(condim=4, type="mesh", rgba=knob_rgba)
    knob_default.add_child(knob_geom)

    robot_default = e.Default(class_='robot')
    robot_joint = e.Joint(damping=robot_damping)
    robot_default.add_children([robot_joint])

    default.add_children([joint_default, wall_default, frame_default, door_default, knob_default, robot_default])

    # Worldbody
    light_condition_list = []
    for i in range(light_n):
        light_condition_list.append(e.Light(
            directional = True,
            diffuse=light_property_list[i]['light_diffuse'],
            pos=light_property_list[i]['light_pos'],
            dir=light_property_list[i]['light_dir']))

    floor_geom = e.Geom(
        name='floor',
        material="Floor",
        pos=[0,0,-0.05],
        size=[40.0,40.0,0.05],
        type='plane')

    #Camera
    camera_list = []
    camera_names = ["camera1", "camera2"]
    for i in range(camera_n):
        camera_list.append(e.Camera(
            name=camera_names[i],
            mode="fixed",
            fovy=camera_fieldview + camera_fieldview_noise[i],
            pos=camera_poses[i],
            euler=camera_ories[i]))
    
    ####### All body obj #######
    body0 = e.Body(
        name='wall_link',
        pos=wall_location,
        childclass='wall')

    body1 = e.Body(
        name='frame_link',
        pos=frame_world_pos,
        childclass='frame')

    body2 = e.Body(
        name='door_link',
        pos=door_pos,
        childclass='door')

    body3 = e.Body(
        name='knob_link'.format(knob_type),
        pos=doorknob_pos,
        childclass='door_knob')

    body4 = e.Body(
        name='{}knob_link'.format(knob_type),
        pos=knob_pos,
        childclass='door_knob')

    worldbody.add_children(light_condition_list)
    worldbody.add_child(floor_geom)
    worldbody.add_children(camera_list)
    worldbody.add_children([body0])

    # ######## Level0 body (origin) ########
    body0_inertial = e.Inertial(
        pos=[0,0,0],
        mass=wall_mass,
        diaginertia=wall_diaginertia)

    body0_geoms = []
    name = 'wall_{}'  
    for i in range(wall_parts_n):
        body0_geoms.append(e.Geom(
            name=name.format(i),
            material=wall_material,
            pos=wall_pos_list[i],
            size=wall_size_list[i],
            type='box',
            euler=frame_euler))

    body0_children_list = [body0_inertial]
    body0_children_list.extend(body0_geoms)
    body0.add_children(body0_children_list)
    body0.add_children([body1])

    ######## Level1 body (Frame) ########
    body1_inertial = e.Inertial(
        pos=[0,0,0],
        mass=frame_mass,
        diaginertia=frame_diaginertia
    )

    body1_geoms = []
    # door stopper to define the pull/push opening direction
    body1_geoms.append(e.Geom(
        name='doorstopper',
        material='Metal',
        pos=doorstopper_pos,
        size=doorstopper_size,
        type='box',
        euler=frame_euler))

    # Actual frame
    name = 'door_frame_{}'
    for i in range(frame_parts_n):
        body1_geoms.append(e.Geom(
            name=name.format(i),
            material=frame_material,
            pos=frame_pos_list[i],
            size=frame_size_list[i],
            type='box',
            euler=frame_euler))

    body1_children_list = [body1_inertial]
    body1_children_list.extend(body1_geoms)
    body1.add_children(body1_children_list)
    body1.add_children([body2])

    ######## Level2 body (Door) ########
    body2_joints = e.Joint(
        name='hinge0',
        type='hinge',
        axis=[0, 0, 1],
        armature=door_frame_armature,
        stiffness=door_frame_spring,
        damping=door_frame_damper,
        frictionloss=door_frame_frictionloss,
        limited=door_frame_limited,
        range=door_frame_range,
        pos=door_frame_joint_pos
    )

    body2_inertial = e.Inertial(
        pos=[0, door_width/2.0-door_width*knob_horizontal_location_ratio, door_height/2.0-knob_height],
        mass=door_mass,
        diaginertia=door_diaginertia
    )
    body2_geoms = [
        e.Geom(
        name='door0',
        material=door_material,
        pos=[0, door_width/2.0-door_width*knob_horizontal_location_ratio, door_height/2.0-knob_height],
        size=[door_thickness/2.0, door_width/2.0, door_height/2.0*0.99],
        type="box",
        euler=door_euler
        )]
    
    body2_children_list = [body2_joints]
    body2_children_list.extend(body2_geoms)
    body2_children_list.extend([body2_inertial])
    body2.add_children(body2_children_list)
    body2.add_children([body3])

    ######## Level3 body (Knob Wrapper) ########
    body3_joints = []

    axes = [[0,1,0],[0,0,1]]
    ranges = [[-0.2, 0.3], [-0.5,0.5]]
    for i in range(2):
        body3_joints.append(
        e.Joint(
            name='target{}'.format(i),
            type='slide',
            axis=axes[i],
            armature=0,
            stiffness=0,
            damping=30000,
            frictionloss=0,
            limited="true",
            range=ranges[i]))

    if knob_type != "pull":
        body3_joints.append(e.Joint(
            name='hinge1',
            type='hinge',
            axis=[1, 0, 0],
            armature=knob_door_armature,
            stiffness=knob_door_spring,
            damping=knob_door_damper,
            frictionloss=knob_door_frictionloss,
            limited=knob_door_limited,
            range=knob_door_range,
            pos=knob_door_joint_pos))

    body3_inertial = e.Inertial(
        pos=[0, 0, 0],
        mass=1,
        diaginertia=[0.001,0.001,0.001])

    body3.add_child(body3_inertial)
    body3.add_children(body3_joints)
    body3.add_children([body4])

    ######## Level4 body (Knob Link) ########
    body4_geoms = []
    name = 'door_knob_{}'
    mesh = 'door_knob_{}'
    knobparts_start_idx = 1
    for i in range(knobparts_start_idx ,knob_parts_n+1):
        body4_geoms.append(e.Geom(
            name=name.format(i),
            material=doorknob_material,
            mesh=mesh.format(i),
            euler=knob_euler,
            friction=knob_surface_friction))    
    if knob_type != "pull":
        body4_geoms.append(
            e.Geom(
            name='knob_latch',
            material=material_name_list[randrange(0,3)],
            pos=[-(latch_gap/2.0+door_thickness*1.5), 0, 0],
            size=[latch_thickness/2.0, latch_width, latch_height],
            type="box",
            euler=[0,0,0]))
    body4_inertial = e.Inertial(
        pos=[-(latch_gap/2.0+door_thickness*2.1), 0, 0],
        mass=knob_mass,
        diaginertia=knob_diaginertia)

    body4_children_list = body4_geoms 
    if knob_type != "pull":
        body4_children_list.extend([body4_inertial])
    body4.add_children(body4_children_list)
    
    ######## Write file to Output folder ########
    model_xml = mujoco.xml()
    name = '{0}_{1}_{2}.xml'
    output_path = os.path.join(args.output_dirname, '{0}_{1}{2}'.format(knob_type, args.robot_type, args.output_name))
    try:
        os.makedirs(output_path)
    except OSError:
        pass
    output_filename = os.path.join(output_path, name.format(dataset_file, knob_type, args.robot_type))
    if knob_type== "lever" and knob_parts_n < 4:
        # print("parts are too less!!", dataset_file)
        pass
    else:
        with open(output_filename, 'w') as fh:
            fh.write(model_xml)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arguments for random generator')
    parser.add_argument('--knob-type', default='', help='Choose from lever, round or pull')
    parser.add_argument('--robot-type', default='floatinghook', help='Choose from "gripper","hook","floatinghook","floatinggripper", "mobile_gripper", "mobile_hook" ')
    parser.add_argument('--input-dirname', type=str, default='./door/{}knobs', help='knob path')
    parser.add_argument('--output-dirname', type=str, default='./world', help='world path')
    parser.add_argument('--output-name', type=str, default=None, help='folder name under world path')
    parser.add_argument('--pulldoor-ratio', type=float, default=1.0, help='ratio of door that opens by pulling.')
    parser.add_argument('--righthinge-ratio', type=float, default=1.0, help='ratio of door that has hinge on right side.')
    args = parser.parse_args()

    # Generate the ordered knob if argument for "knob_type"
    if args.knob_type:
        assert args.knob_type in ['lever', 'round', 'pull']
        knob_type = args.knob_type
        pbar = tqdm(os.listdir(args.input_dirname.format(args.knob_type)))
        for dataset_file in pbar:
            main(dataset_file)
    # Generate all type of knob if no argument for "knob_type"
    else:
        knob_types = ['lever','round','pull']
        for knob_type in knob_types:
            print("Generating knob:type:{}".format(knob_type))
            pbar = tqdm(os.listdir(args.input_dirname.format(knob_type)))
            for dataset_file in pbar:
                main(dataset_file)