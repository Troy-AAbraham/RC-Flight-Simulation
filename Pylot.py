import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import graphics
from sim import simi





def main():
    #initialize pygame module and set window
    pygame.init()

    #creates instance of the aircraft simulator class
    aircraft_sim = simi()
    aircraft_sim.load_file(filename = "11.24_input_test.json")
    aircraft_sim.init_states()
    aircraft_sim.trim_func(trim = 0)

    #DISPLAY WINDOW SIZE. CHANGE TO SUIT YOUR SCREEN IF NECESSARY
    width, height = 1600,847
    pygame.display.set_icon(pygame.image.load('res/gameicon.jpg'))
    screen = pygame.display.set_mode((width,height), HWSURFACE|OPENGL|DOUBLEBUF)
    pygame.display.set_caption("Pylot")
    glViewport(0,0,width,height)
    glEnable(GL_DEPTH_TEST)
    default_view = np.identity(4)

    #boolean variables for camera view, lose screen, and pause
    VIEW = 0
    LOSE = False
    PAUSE = False
    KEYBOARD = False
    DATA = True

    #SIMULATION FRAMERATE
    #pygame limits framerate to this value. Increasing framerate improves accuracy of simulation (max limit = 60) but may cause some lag in graphics
    target_framerate = 60

    #initialize graphics objects
    #loading screen is rendered and displayed while the rest of the objects are read and prepared
    glClearColor(0.,0.,0.,1.0)
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    loading = graphics.Text(150)
    loading.draw(-0.2,-0.05,"Loading...",(0,255,0,1))
    pygame.display.flip()

    #initialize game over screen
    gameover = graphics.Text(150)

    #initialize graphics aircraft
    graphics_aircraft = graphics.Mesh("res/Cessna_small.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/cessna_texture_small.jpg",width,height)
    graphics_aircraft.set_orientation([0.,0.,0.,0.])
    graphics_aircraft.set_position([0.,0.,-50.])

    #initialize graphics sky
    graphics_sky = graphics.Mesh("res/sky2.obj","shaders/sky.vs","shaders/sky.fs","res/clouds2.jpeg",width,height)
    graphics_sky.set_orientation([-1.,0.,0.,0.])
    graphics_sky.set_position([0.,0.,0.])

    #initialize graphics bench
    graphics_bench = graphics.Mesh("res/wood_bench.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/Oak.jpg",width,height)
    graphics_bench.set_orientation([-1.,0.,0.,0.])
    graphics_bench.set_position([20.,2.5,0.])

#initialize graphics tent
    graphics_tent = graphics.Mesh("res/tent.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/Tent.jpeg",width,height)
    graphics_tent.set_orientation([-1.,0.,0.,0.])
    graphics_tent.set_position([20.,2.5,0.])

    #initialize graphics trees 1 -8
    graphics_tree1 = graphics.Mesh("res/spruce.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/tree_texture.jpg",width,height)
    graphics_tree1.set_orientation([-1.,0.,0.,0.])
    graphics_tree1.set_position([-466.,-250.,0.])

    graphics_tree2 = graphics.Mesh("res/spruce.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/tree_texture.jpg",width,height)
    graphics_tree2.set_orientation([-1.,0.,0.,0.])
    graphics_tree2.set_position([-466.,250.,0.])

    graphics_tree3 = graphics.Mesh("res/spruce.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/tree_texture.jpg",width,height)
    graphics_tree3.set_orientation([-1.,0.,0.,0.])
    graphics_tree3.set_position([466.,-250.,0.])

    graphics_tree4 = graphics.Mesh("res/spruce.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/tree_texture.jpg",width,height)
    graphics_tree4.set_orientation([-1.,0.,0.,0.])
    graphics_tree4.set_position([466.,250.,0.])

    graphics_tree5 = graphics.Mesh("res/spruce.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/tree_texture.jpg",width,height)
    graphics_tree5.set_orientation([-1.,0.,0.,0.])
    graphics_tree5.set_position([-240.,-80.,0.])

    graphics_tree6 = graphics.Mesh("res/spruce.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/tree_texture.jpg",width,height)
    graphics_tree6.set_orientation([-1.,0.,0.,0.])
    graphics_tree6.set_position([-240.,-10.,0.])

    graphics_tree7 = graphics.Mesh("res/spruce.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/tree_texture.jpg",width,height)
    graphics_tree7.set_orientation([-1.,0.,0.,0.])
    graphics_tree7.set_position([170.,-80.,0.])

    graphics_tree8 = graphics.Mesh("res/spruce.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/tree_texture.jpg",width,height)
    graphics_tree8.set_orientation([-1.,0.,0.,0.])
    graphics_tree8.set_position([170.,-10.,0.])

    #initialize field
    field = graphics.Mesh("res/field2.obj","shaders/field.vs","shaders/field.fs","res/field_texture.jpg",width,height)
    field.set_position(np.array([0.,0.,0.75]))

    #initialize graphics rc field
    rc_field = graphics.Mesh("res/field_landing.obj","shaders/field.vs","shaders/field.fs","res/field_landing.jpeg",width,height)
    rc_field.set_position(np.array([0.,0.,-0.35]))

    #initialize graphics rc runway
    landing_strip = graphics.Mesh("res/landing.obj","shaders/sky.vs","shaders/sky.fs","res/landing.jpg",width,height)
    landing_strip.set_orientation([-1.,0.,0.,0.])
    landing_strip.set_position(np.array([-30.,-40.,-0.5]))

    #initialize graphics aircraft shadow
    shadow = graphics.Mesh("res/shadow.obj","shaders/sky.vs","shaders/sky.fs","res/shadow.jpg",width,height)
    shadow.set_orientation([-1.,0.,0.,0.])
    shadow.set_position(np.array([0.,0.,-50.]))

    #initialize HUD
    HUD = graphics.HeadsUp(width, height)

    #initialize flight data overlay
    data = graphics.FlightData()

    #initialize camera object
    cam = graphics.Camera()

    # interferance function defined for a box
    def box(plane_v):
        x_up = 26.
        x_low = 14.
        y_up = 7.5
        y_low = -2.5
        z_up = -15
        z_low = 0
        if (x_low <= plane_v[0] <= x_up) and (y_low <= plane_v[1] <= y_up) and (z_up <= plane_v[2] <= z_low):
            return True
        else:
            return False

    # interferance function defined for a cone
    def cone(plane_v, obj_pos):
        if plane_v[2] > -23.:
            r_allow = (23. + plane_v[2])/6.6
            r_actual = np.sqrt((obj_pos[0] - plane_v[0])*(obj_pos[0] - plane_v[0]) + (obj_pos[1] - plane_v[1])*(obj_pos[1] - plane_v[1]))
            if r_actual <= r_allow:
                return True
            else:
                return False
        else:
            return False




    #initialize other pygame elementsw
    if pygame.joystick.get_count()>0.:
        joy = pygame.joystick.Joystick(0)
        joy.init()
    else:
        KEYBOARD = True
        thr = 0.
        UP = False
        DOWN = False
        RIGHT = False
        LEFT = False
        WW = False
        SS = False
        AA = False
        DD = False
        tt = False
        BB = False

    #DEFLECTION LIMITS FOR CONTROL SURFACES
    d_ail = 15.
    d_ele = 15.
    d_rud = 15.
    brake = 0.
    flag_trim = True

    #clock object for tracking frames and timestep
    clock = pygame.time.Clock()


    #ticks clock before starting game loop
    clock.tick_busy_loop()

    #game loop
    while True:
        #event loop checks for game inputs such as joystick and keyboard commands
        for event in pygame.event.get():
            if event.type == QUIT:
                return

            if KEYBOARD == False:
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button ==12:
                        cam.pos_storage.clear()
                        cam.up_storage.clear()
                        cam.target_storage.clear()

            else:
                if event.type == pygame.KEYDOWN:

                    if event.key == pygame.K_UP:
                        UP = True
                    if event.key == pygame.K_DOWN:
                        DOWN = True
                    if event.key == pygame.K_LEFT:
                        LEFT = True
                    if event.key == pygame.K_RIGHT:
                        RIGHT = True

                    if event.key == pygame.K_w:
                        WW = True
                    if event.key == pygame.K_s:
                        SS = True
                    if event.key == pygame.K_a:
                        AA = True
                    if event.key == pygame.K_d:
                        DD = True
                    if event.key == pygame.K_b:
                        BB = True
                    if event.key == pygame.K_SPACE:
                        VIEW += 1
                        if VIEW > 3:
                            VIEW = 0
                        cam.pos_storage.clear()
                        cam.up_storage.clear()
                        cam.target_storage.clear()
                    if event.key == pygame.K_t:
                        flag_trim = not flag_trim
                        aircraft_sim.new_trim()

                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_UP:
                        UP = False
                    if event.key == pygame.K_DOWN:
                        DOWN = False
                    if event.key == pygame.K_LEFT:
                        LEFT = False
                    if event.key == pygame.K_RIGHT:
                        RIGHT = False

                    if event.key == pygame.K_w:
                        WW = False
                    if event.key == pygame.K_s:
                        SS = False
                    if event.key == pygame.K_a:
                        AA = False
                    if event.key == pygame.K_d:
                        DD = False
                    if event.key == pygame.K_b:
                        BB = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_i:
                    DATA = not DATA
                #pause simulation
                if event.key == pygame.K_p:
                    PAUSE = not PAUSE
                #quit game
                if event.key == pygame.K_q:
                    return

        #maintains framerate even if sim is paused
        if PAUSE == True:
            clock.tick(target_framerate)

        #if game is not paused, runs sim
        if PAUSE == False:
            #set default background color for sky
            glClearColor(0.65,1.0,1.0,1.0)
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

            #timestep for simulation is based on framerate
            t= clock.tick(target_framerate)/1000.

            # if joystick is being used, gets joystick input and creates control_state dicitonary
            if KEYBOARD == False:
                flag_trim = False
                control_state = {
                    "aileron": (joy.get_axis(3)**3)*-d_ail,
                    "elevator": (joy.get_axis(4)**3)*d_ele,
                    "rudder": (joy.get_axis(0)**3)*-d_rud,
                    "throttle": (joy.get_axis(1)+1.)*0.5,
                    "flaps": 0.
                    }
            # if joystick is not being used, gets keyboard input and creates control_state dictionary
            else:

                if UP or DOWN or LEFT or RIGHT or WW or SS or AA or DD:
                    flag_trim = False
                if UP == True and DOWN == False:
                    ele = 1.
                elif UP == False and DOWN == True:
                    ele = -1.
                else:
                    ele = 0.
                if LEFT == True and RIGHT == False:
                    ail = 1.
                elif LEFT == False and RIGHT == True:
                    ail = -1.
                else:
                    ail = 0.
                if AA == True and DD == False:
                    rud = 1.
                elif AA == False and DD == True:
                    rud = -1.
                else:
                    rud = 0.
                if WW == True and SS == False and thr<=1.0:
                    thr += 0.01
                elif WW == False and SS == True and thr>=0.0:
                    thr -= 0.01
                if BB == True and brake<=5.0:
                    brake = 1.
                else:
                    brake = 0.1

                # trim and retrim conditions
                if flag_trim == True:
                    thr_temp = aircraft_sim.tau
                    if thr_temp >= 1.0:
                        # if throttle too large for trim, doesnt trim
                        control_state = {
                            "aileron": ail*d_ail,
                            "elevator": ele*d_ele,
                            "rudder": rud*d_rud,
                            "throttle": thr,
                            "flaps": 0.,
                            "brake": brake
                            }
                    else:
                        thr = thr_temp
                        control_state = {
                            "aileron": aircraft_sim.da*(180/np.pi),
                            "elevator": aircraft_sim.de*(180/np.pi),
                            "rudder": aircraft_sim.dr*(180/np.pi),
                            "throttle": thr,
                            "flaps": 0.
                        }
                else:
                    control_state = {
                        "aileron": ail*d_ail,
                        "elevator": ele*d_ele,
                        "rudder": rud*d_rud,
                        "throttle": thr,
                        "flaps": 0.,
                        "brake": brake
                    }


            #SIMULATION CALCULATIONS GO BELOW HERE FOR EACH TIME STEP
            #IT IS RECOMMENDED THAT YOU MAKE AN OBJECT FOR THE SIMULATION AIRCRAFT AND CREATE A FUNCTION IN SAID OBJECT TO CALCULATE THE NEXT TIME STEP.
            #THIS FUNCTION CAN THEN BE CALLED HERE

            if flag_trim == False:

                aircraft_sim.de0 = control_state["elevator"]*(np.pi/180.)
                aircraft_sim.da0 = control_state["aileron"]*(np.pi/180.)
                aircraft_sim.dr0 = control_state["rudder"]*(np.pi/180.)
                aircraft_sim.tau = control_state["throttle"]
                aircraft_sim.cf = control_state["brake"]

            aircraft_sim.trimmed = flag_trim
            aircraft_sim.run_sim(t)

            u, v, w, p, q, r, x, y, z, e0, ex, ey, ez = aircraft_sim.state


            #INPUT POSITION, ORIENTATION, AND VELOCITY OF AIRCRAFT INTO THIS DICTIONARY WHICH WILL THEN UPDATE THE GRAPHICS
            aircraft_condition = {
                "Position":np.array([x,y,z]),#input position of form [x,y,z]
                "Orientation":np.array([e0,ex,ey,ez]),#input orientation in quaternion form [e0,ex,ey,ez]
                "Velocity":np.array([u,v,w]) #input Velocity of form [u,v,w]
            }
            flight_data = {
                "Graphics Time Step": t,#sec
                "Physics Time Step":aircraft_sim.dt,#sec
                "Airspeed": aircraft_sim.V,#feet/sec
                "AoA":aircraft_sim.alpha*(180/np.pi),#deg
                "Sideslip":aircraft_sim.beta*(180/np.pi),#deg
                "Altitude":-z,#feet
                "Latitude":0. ,#deg
                "Longitude":0. ,#deg
                "Time":aircraft_sim.ti,#sec
                "Bank":aircraft_sim.phi*(180/np.pi),#deg
                "Elevation":aircraft_sim.theta*(180/np.pi),#deg
                "Heading":aircraft_sim.heading*(180/np.pi),#deg
                "Gnd Speed":aircraft_sim.gnd_V ,#feet/sec
                "Gnd Track":aircraft_sim.heading*(180/np.pi) ,#deg
                "Climb":-60.*aircraft_sim.Vzf, #feet/min
                "Throttle":control_state["throttle"]*100 ,#%
                "Elevator":control_state["elevator"] ,#deg
                "Ailerons":control_state["aileron"] ,#deg
                "Rudder":control_state["rudder"] ,#deg
                "Flaps":control_state["flaps"] ,#deg
                "Axial G-Force":(aircraft_sim.X + aircraft_sim.Txb)/aircraft_sim.W ,#g's
                "Side G-Force":aircraft_sim.Y/aircraft_sim.W ,#g's
                "Normal G-Force":aircraft_sim.Z/aircraft_sim.W ,#g's
                "Roll Rate":p*(180/np.pi) ,#deg/s
                "Pitch Rate":q*(180/np.pi) ,#deg/s
                "Yaw Rate":r*(180/np.pi)#deg/s
            }

            #apply position and orientation to graphics
            graphics_aircraft.set_orientation(graphics.swap_quat(aircraft_condition["Orientation"]))
            graphics_aircraft.set_position(aircraft_condition["Position"])

            # renders shadow if below 40 ft altitude
            if z > -40.:
                shadow_orient = aircraft_sim.Euler2Quat([0.,0.,aircraft_sim.heading])

                shadow.set_orientation(graphics.swap_quat(shadow_orient))
                shadow.set_position([x,y,-0.65])

            #interferance test for each body fixed component vector
            L1 = box(aircraft_condition["Position"])
            L2 = cone(aircraft_condition["Position"], [-466.,-250.,0.])
            L3 = cone(aircraft_condition["Position"], [-466.,250.,0.])
            L4 = cone(aircraft_condition["Position"], [466.,-250.,0.])
            L5 = cone(aircraft_condition["Position"], [466.,250.,0.])
            L6 = cone(aircraft_condition["Position"], [-240.,-80.,0.])
            L7 = cone(aircraft_condition["Position"], [-240.,-10.,0.])
            L8 = cone(aircraft_condition["Position"], [170.,-80.,0.])
            L9 = cone(aircraft_condition["Position"], [170.,10.,0.])
            L1L = box(aircraft_sim.wing_l)
            L2L = cone(aircraft_sim.wing_l, [-466.,-250.,0.])
            L3L = cone(aircraft_sim.wing_l, [-466.,250.,0.])
            L4L = cone(aircraft_sim.wing_l, [466.,-250.,0.])
            L5L = cone(aircraft_sim.wing_l, [466.,250.,0.])
            L6L = cone(aircraft_sim.wing_l, [-240.,-80.,0.])
            L7L = cone(aircraft_sim.wing_l, [-240.,-10.,0.])
            L8L = cone(aircraft_sim.wing_l, [170.,-80.,0.])
            L9L = cone(aircraft_sim.wing_l, [170.,10.,0.])
            L1R = box(aircraft_sim.wing_r)
            L2R = cone(aircraft_sim.wing_r, [-466.,-250.,0.])
            L3R = cone(aircraft_sim.wing_r, [-466.,250.,0.])
            L4R = cone(aircraft_sim.wing_r, [466.,-250.,0.])
            L5R = cone(aircraft_sim.wing_r, [466.,250.,0.])
            L6R = cone(aircraft_sim.wing_r, [-240.,-80.,0.])
            L7R = cone(aircraft_sim.wing_r, [-240.,-10.,0.])
            L8R = cone(aircraft_sim.wing_r, [170.,-80.,0.])
            L9R = cone(aircraft_sim.wing_r, [170.,10.,0.])

            #interference and vertical velocity endgame conditions
            if L1 or L2 or L3 or L4 or L5 or L6 or L7 or L8 or L9 or L1L or L2L or L3L or L4L or L5L or L6L or L7L or L8L or L9L or L1R or L2R or L3R or L4R or L5R or L6R or L7R or L8R or L9R:
                LOSE = True
            if aircraft_sim.Vzf > 12 and z > -2.5:
                LOSE = True

            #if you get a game over, display lose screen
            if LOSE == True:
                glClearColor(0,0,0,1.0)
                glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

                gameover.draw(-0.2,-0.05,"Game Over",(0,255,0,1))
                PAUSE = True

            #otherwise, render graphics
            #stationary view
            if VIEW == 0:

                view =  cam.stationary_view(graphics_aircraft)

                graphics_aircraft.set_view(view)
                graphics_aircraft.render()

                field.set_view(view)
                field.render()

                rc_field.set_view(view)
                rc_field.render()

                landing_strip.set_view(view)
                landing_strip.render()

                graphics_sky.set_view(view)
                graphics_sky.render()

                graphics_bench.set_view(view)
                graphics_bench.render()

                graphics_tent.set_view(view)
                graphics_tent.render()

                graphics_tree1.set_view(view)
                graphics_tree1.render()

                graphics_tree2.set_view(view)
                graphics_tree2.render()

                graphics_tree3.set_view(view)
                graphics_tree3.render()

                graphics_tree4.set_view(view)
                graphics_tree4.render()

                graphics_tree5.set_view(view)
                graphics_tree5.render()

                graphics_tree6.set_view(view)
                graphics_tree6.render()

                graphics_tree7.set_view(view)
                graphics_tree7.render()

                graphics_tree8.set_view(view)
                graphics_tree8.render()

                if aircraft_sim.zfpr > -40.:
                    shadow.set_view(view)
                    shadow.render()



                if DATA == True:
                    data.render(flight_data)

            #3rd behind view
            elif VIEW == 1:

                #get view matrix and render scene
                view = cam.third_view(graphics_aircraft)

                graphics_aircraft.set_view(view)
                graphics_aircraft.render()

                field.set_view(view)
                field.render()

                rc_field.set_view(view)
                rc_field.render()

                landing_strip.set_view(view)
                landing_strip.render()

                graphics_sky.set_view(view)
                graphics_sky.render()

                graphics_bench.set_view(view)
                graphics_bench.render()

                graphics_tent.set_view(view)
                graphics_tent.render()

                graphics_tree1.set_view(view)
                graphics_tree1.render()

                graphics_tree2.set_view(view)
                graphics_tree2.render()

                graphics_tree3.set_view(view)
                graphics_tree3.render()

                graphics_tree4.set_view(view)
                graphics_tree4.render()

                graphics_tree5.set_view(view)
                graphics_tree5.render()

                graphics_tree6.set_view(view)
                graphics_tree6.render()

                graphics_tree7.set_view(view)
                graphics_tree7.render()

                graphics_tree8.set_view(view)
                graphics_tree8.render()

                if aircraft_sim.zfpr > -40.:
                    shadow.set_view(view)
                    shadow.render()

                if DATA == True:
                    data.render(flight_data)
            #side view
            elif VIEW == 2:

                #get view matrix and render scene
                view = cam.side_view(graphics_aircraft)

                graphics_aircraft.set_view(view)
                graphics_aircraft.render()

                field.set_view(view)
                field.render()

                rc_field.set_view(view)
                rc_field.render()

                landing_strip.set_view(view)
                landing_strip.render()

                graphics_sky.set_view(view)
                graphics_sky.render()

                graphics_bench.set_view(view)
                graphics_bench.render()

                graphics_tent.set_view(view)
                graphics_tent.render()

                graphics_tree1.set_view(view)
                graphics_tree1.render()

                graphics_tree2.set_view(view)
                graphics_tree2.render()

                graphics_tree3.set_view(view)
                graphics_tree3.render()

                graphics_tree4.set_view(view)
                graphics_tree4.render()

                graphics_tree5.set_view(view)
                graphics_tree5.render()

                graphics_tree6.set_view(view)
                graphics_tree6.render()

                graphics_tree7.set_view(view)
                graphics_tree7.render()

                graphics_tree8.set_view(view)
                graphics_tree8.render()

                if aircraft_sim.zfpr > -40.:
                    shadow.set_view(view)
                    shadow.render()

                if DATA == True:
                    data.render(flight_data)

            #cockpit view
            elif VIEW == 3:

                view = cam.cockpit_view(graphics_aircraft)

                field.set_view(view)
                field.render()

                rc_field.set_view(view)
                rc_field.render()

                landing_strip.set_view(view)
                landing_strip.render()

                graphics_sky.set_view(view)
                graphics_sky.render()

                graphics_bench.set_view(view)
                graphics_bench.render()

                graphics_tent.set_view(view)
                graphics_tent.render()

                graphics_tree1.set_view(view)
                graphics_tree1.render()

                graphics_tree2.set_view(view)
                graphics_tree2.render()

                graphics_tree3.set_view(view)
                graphics_tree3.render()

                graphics_tree4.set_view(view)
                graphics_tree4.render()

                graphics_tree5.set_view(view)
                graphics_tree5.render()

                graphics_tree6.set_view(view)
                graphics_tree6.render()

                graphics_tree7.set_view(view)
                graphics_tree7.render()

                graphics_tree8.set_view(view)
                graphics_tree8.render()

                if DATA == True:
                    data.render(flight_data)
                HUD.render(aircraft_condition,view)



            #update screen display
            pygame.display.flip()


if __name__ == "__main__":
    main()
