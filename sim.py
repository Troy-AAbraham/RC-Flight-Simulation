from math import cos, sin, atan, atan2, asin, sqrt, exp, pi
import numpy as np
import matplotlib.pyplot as plt
import json
from mpl_toolkits import mplot3d



class simi:

    """Flight Simulator Class

    Methods of significance
    -----------------------
    load_file():
              Function that loads simulation, aircraft, and reference state values.
    init_states():
              Initializes aircraft coefficients for drag and reference state
              thrust and lift.
    trim_func():
              Solves the system of equations for trim values at the flight
              conditions specified through json input. Initializes the simulator
              state values at these trim conditions.
    run_sim():
              Updates flight simulator state values at each timestep over the
              time interval provided.
    aero():
              Solves for aerodynamic forces and moments at each timestep (RK4 step)

    Notes
    ------
    There is a significant number of additonal methods included in this class.
    These methods are organized by application and include their own descriptions.
    """

#-----------------------------------------------------------------------------#
#SIMULATOR INITIALIZATION FUNCTIONS

    def __init__(self):
        return

    def load_file(self, filename):

        '''Loads aircraft parameters from json input file

        Parameters
        -----------
        filename: string
            name of json file input
        '''

        json_vals=open(filename).read()
        vals_temp = json.loads(json_vals)

        #simulation timestep and density conditions
        self.dt = vals_temp["simulation"]["timestep"]
        self.t_final = vals_temp["simulation"]["total_time"]
        self.constant_density = vals_temp["simulation"]["constant_density"]

        #aircraft wing area and span
        self.Sw = vals_temp["aircraft"]["wing_area"]
        self.bw = vals_temp["aircraft"]["wing_span"]

        #aircraft thrust values
        self.zT0 = vals_temp["aircraft"]["thrust"]["offset"]
        self.T0 = vals_temp["aircraft"]["thrust"]["T0"]
        self.T1 = vals_temp["aircraft"]["thrust"]["T1"]
        self.T2 = vals_temp["aircraft"]["thrust"]["T2"]
        self.a = vals_temp["aircraft"]["thrust"]["a"]

        #aircraft initial conditions
        self.V0 = vals_temp["initial"]["airspeed"]
        self.alt = vals_temp["initial"]["altitude"]
        self.W = vals_temp["initial"]["weight"]
        self.gamma0 = vals_temp["initial"]["climb"]*(pi/180.)
        self.phi0 = vals_temp["initial"]["bank"]*(pi/180.)
        self.psi0 = vals_temp["initial"]["heading"]*(pi/180.)

        #aircraft reference flight values
        self.V_ref = vals_temp["reference"]["airspeed"]
        self.rho_ref = vals_temp["reference"]["density"]
        self.de_ref = vals_temp["reference"]["elevator"]*(pi/180.)
        self.lift_ref = vals_temp["reference"]["lift"]

        #aircraft moments of inertia
        self.Ixx = vals_temp["reference"]["Ixx"]
        self.Iyy = vals_temp["reference"]["Iyy"]
        self.Izz = vals_temp["reference"]["Izz"]
        self.Ixy = vals_temp["reference"]["Ixy"]
        self.Ixz = vals_temp["reference"]["Ixz"]
        self.Iyz = vals_temp["reference"]["Iyz"]
        self.hxb = vals_temp["reference"]["hx"]
        self.hyb = vals_temp["reference"]["hy"]
        self.hzb = vals_temp["reference"]["hz"]

        #aircraft force and moment coefficients
        self.CD_ref = vals_temp["reference"]["CD"]
        self.CLa = vals_temp["reference"]["CL,a"]
        self.CDa = vals_temp["reference"]["CD,a"]
        self.CDaa = vals_temp["reference"]["CD,a,a"]
        self.Cma = vals_temp["reference"]["Cm,a"]
        self.CYb = vals_temp["reference"]["CY,b"]
        self.Clb = vals_temp["reference"]["Cl,b"]
        self.Cnb = vals_temp["reference"]["Cn,b"]
        self.CLq = vals_temp["reference"]["CL,q"]
        self.CDq = vals_temp["reference"]["CD,q"]
        self.Cmq = vals_temp["reference"]["Cm,q"]
        self.CYp = vals_temp["reference"]["CY,p"]
        self.Clp = vals_temp["reference"]["Cl,p"]
        self.Cnp = vals_temp["reference"]["Cn,p"]
        self.CYr = vals_temp["reference"]["CY,r"]
        self.Clr = vals_temp["reference"]["Cl,r"]
        self.Cnr = vals_temp["reference"]["Cn,r"]
        self.CLde = vals_temp["reference"]["CL,de"]
        self.CDde = vals_temp["reference"]["CD,de"]
        self.Cmde = vals_temp["reference"]["Cm,de"]
        self.CYda = vals_temp["reference"]["CY,da"]
        self.Clda = vals_temp["reference"]["Cl,da"]
        self.Cnda = vals_temp["reference"]["Cn,da"]
        self.CYdr = vals_temp["reference"]["CY,dr"]
        self.Cldr = vals_temp["reference"]["Cl,dr"]
        self.Cndr = vals_temp["reference"]["Cn,dr"]
        self.CD3 = vals_temp["reference"]["CD3"]

        #set gravitational acceleration, density, and solve for average chord
        self.g = 32.2
        self.rho = self.rho_ref
        self.cb = self.Sw/self.bw

        #solve for reference state lift
        self.ref_co = 0.5*self.rho_ref*self.V_ref*self.V_ref*self.Sw
        self.CL_ref = (self.lift_ref)/(self.ref_co)

        self.z = -self.alt
        self.x = 0.0
        self.y = 0.0

        # wheel component vectors and friction coefficient
        self.w_front = [6,0,1]
        self.w_b_left = [-6,-6,1]
        self.w_b_right = [-6,6,1]
        self.cf = 0.1

    def init_states(self):

        '''Initializes simulator and aircraft values that are fixed for 
        the current reference state

        Notes
        -----------
        CD2: non-linear change in zero-lift drag with lift coefficient
        CD1: linear change in zero-lift drag with lift coefficient
        CD0: zero-lift drag coefficient
        T_ref: reference state thrust
        Cm_ref: reference state pitching moment coefficent
        Cw: coefficient of weight

        Other values that are fixed for the trim solution are initialized here.
        '''

        #initial simulator time
        self.ti = 0.

        #initial airspeed
        self.V = self.V0

        #solve for reference state drag and moment coefficients
        self.CD2 = (self.CDaa/(2*self.CLa*self.CLa))
        self.CD1 = (self.CDa/self.CLa) - 2*self.CD2*self.CL_ref
        self.CD0 = self.CD_ref - self.CD1*self.CL_ref - self.CD2*self.CL_ref*self.CL_ref

        #reference thrust
        self.T_ref = self.ref_co*self.CD_ref

        #reference pitching moment coefficient
        self.Cm_ref = -(self.zT0/self.cb)*self.CD_ref


        #trim state coefficient
        self.coeff1 = 0.5*self.rho*self.V*self.V*self.Sw

        #aircraft coefficient of weight
        self.Cw = self.W/self.coeff1

        #initializes trim state values
        self.tau = 0.
        self.alpha = 0.
        self.beta = 0.
        self.de0 = self.de_ref - self.de_ref
        self.da0 = 0.
        self.dr0 = 0.

        self.phi = self.phi0
        self.psi = self.psi0
        self.gamma = self.gamma0

        #ratios for non dimensionalizing rotation rates
        self.nonc = self.cb/(2*self.V)
        self.nonb = self.bw/(2*self.V)

    def new_trim(self):

        ''' sets current values necessary to re-trim aircraft
        '''

        #trim state coefficient
        self.coeff1 = 0.5*self.rho*self.V*self.V*self.Sw

        #aircraft coefficient of weight
        self.Cw = self.W/self.coeff1

        #initializes trim state values
        self.tau = 0.
        self.alpha = 0.
        self.beta = 0.
        self.de0 = self.de_ref - self.de_ref
        self.da0 = 0.
        self.dr0 = 0.

        #ratios for non dimensionalizing rotation rates
        self.nonc = self.cb/(2*self.V)
        self.nonb = self.bw/(2*self.V)
        vx, vy, vz = self.Body2Fixed([self.upr, self.vpr, self.wpr], [self.e0pr, self.expr, self.eypr, self.ezpr])
        self.gamma = asin(-vz/self.V)
        self.z = self.zfpr
        self.x = self.xfpr
        self.y = self.yfpr
        self.trim_func()
        self.trimmed = True


    def trim_func(self, trim = 0):

        '''Solves the trim function system of equations for trim throttle,
        alpha, beta, and the trim elevator, aileron, and rudder delections.
        Initializes the simulator state values for this trim state.

        Parameters
        -----------
        trim: integer
            Specifies which trim SOE to use. 0 is default for the SOE with
            more terms retained in LHS matrix. 1 specifies the simpler SOE with
            more terms on the RHS (provided as solution in class).

        Notes
        -----------
        The matrix and arrays that are used to solve the system of equations
        are included in this class as seperate member functions.  See the 
        'Equilibrium State Functions' section for their definitions.
        '''

        #Initialize loop conditional value, error between previous trim values
        diff = 1.

#        #number of iterations to convergence
#        self.count = 0

        # Solves the trim SOE until sufficient convergence is met
        while (diff > 1e-14):

#            self.count = self.count + 1

            #updates previous trim state values for comparison
            prev = np.array([self.tau, self.alpha, self.beta, self.de0, self.da0, self.dr0])

            #calls member function to update the velocities u,v,w
            self.update_vel(self.V)

            #solves for elevation angle at current trim estimate
            self.theta = self.theta_solve2()

            #solves for equilibrium turning rate
            self.omega = self.g*((sin(self.phi)*cos(self.theta))/(self.wo*sin(self.theta) + 
                                                                   self.uo*cos(self.phi)*cos(self.theta)))

            #trim state rotation rates from turning rate, bank, and elevation angle
            p0 = self.omega*(-sin(self.theta))
            q0 = self.omega*(sin(self.phi)*cos(self.theta))
            r0 = self.omega*(cos(self.phi)*cos(self.theta))

            #non-dimensionalize rotation rates
            pb0 = p0*self.nonb
            qb0 = q0*self.nonc
            rb0 = r0*self.nonb

            #current estimate for coefficient of thrust
            CT = (((self.rho/self.rho_ref)**self.a)*(self.T0 + 
                              self.T1*self.V + self.T2*self.V*self.V))/self.coeff1

            #current estimate for coefficient of lift
            self.CL = (self.CL_ref + self.CLa*self.alpha + self.CLq*qb0 + 
               self.CLde*(self.de0 - self.de_ref))

            #current estimate for coefficient of sideforce
            self.Cs = (self.CYb*self.beta + self.CYp*pb0 + self.CYr*rb0 +
                       self.CYda*self.da0 + self.CYdr*self.dr0)

            #current estimate for coefficient of drag
            self.CD = (self.CD0 + self.CD1*self.CL + self.CD2*self.CL*self.CL 
                       + self.CD3*self.Cs*self.Cs + self.CDq*qb0 + self.CDde*(self.de0 - self.de_ref))

            #variable defined to simplify the system of equations for trim
            self.X1 = (self.CD0 + self.CD1*(self.CL_ref + self.CLq*qb0) + 
                       self.CD2*self.CL*self.CL + self.CD3*self.Cs*self.Cs + 
                       self.CDq*qb0)

            #ratio of velocities found in the trim system of equations
            self.ur = self.uo/self.V
            self.vr = self.vo/self.V
            self.wr = self.wo/self.V

            #two methods for solving the trim state are available, see function
            #docstring for explanation
            if trim == 0:
                #my solution
                mat = self.trim_mat(CT, pb0, qb0, rb0)
                array = self.trim_array(p0, q0, r0, pb0, qb0, rb0)
            elif trim == 1:
                #solution provided in class
                mat = self.trim_mat2(CT, pb0, qb0, rb0)
                array = self.trim_array2(p0, q0, r0, pb0, qb0, rb0)
            else:
                print("Trim input not valid.")
                break

            #solves SOE
            sol = np.linalg.solve(mat,array)

            #parses solution into the respective trim values
            self.tau = float(sol[0])
            self.alpha = float(sol[1])
            self.beta = float(sol[2])
            self.de0deref = float(sol[3])
            self.de0 = float(self.de0deref + self.de_ref)
            self.da0 = float(sol[4])
            self.dr0 = float(sol[5])

            #updates and compares the largest difference between previous trim
            #values and the most recent results
            cur = np.array([self.tau, self.alpha, self.beta, self.de0, self.da0, self.dr0]) 
            d = np.array([abs(prev[i ]- cur[i]) for i in range(len(cur))])
            diff = float(max(d))

        # #writes trim results to file
        # self.write_trim()

        # #outputs to console the final resulting trim values
        # self.print_trim()

        #updates all trim values for a final time, ensuring most converged solution
        self.theta = self.theta_solve2()
        self.omega = self.g*((sin(self.phi)*cos(self.theta))/(self.wo*sin(self.theta) + 
                                                               self.uo*cos(self.phi)*cos(self.theta)))

        p0 = self.omega*(-sin(self.theta))
        q0 = self.omega*(sin(self.phi)*cos(self.theta))
        r0 = self.omega*(cos(self.phi)*cos(self.theta))

        self.update_vel(self.V)

        #set simulator states to the trim state solutions
        #velocities
        self.upr = self.uo
        self.vpr = self.vo
        self.wpr = self.wo

        #bank angle
        self.phi = self.phi

        #heading
        self.psi =  self.psi
        self.psiprev = self.psi

        #rotation rates
        self.ppr = p0
        self.qpr = q0
        self.rpr = r0

        #positions
        self.xfpr = self.x
        self.yfpr = self.y
        self.zfpr = self.z

        #quaternions
        self.e0pr, self.expr, self.eypr, self.ezpr = self.Euler2Quat([self.phi,
                                                                      self.theta,
                                                                      self.psi])
        self.da = self.da0
        self.de = self.de0
        self.dr = self.dr0

        self.trimmed = True

#-----------------------------------------------------------------------------#
#MAIN SIMULATOR FUNCTION
    def set_timestep(self, t_g):
        return

    def run_sim(self, t_g):

        '''Run the flight simulator for the given loop condition. Currently 
        runs for set amount of time. Calls all necessary member functions to
        update the current aircraft simulator state. Also prints/plots/ and
        writes state values if desired.
        '''

        # updates the earth fixed positions of the wheels and wings

        self.w_f = self.Body2Fixed(self.w_front,[self.e0pr,self.expr,self.eypr,self.ezpr])
        self.w_f = [self.xfpr + self.w_f[0], self.yfpr + self.w_f[1], self.zfpr + self.w_f[2]]

        self.w_b_l = self.Body2Fixed(self.w_b_left,[self.e0pr,self.expr,self.eypr,self.ezpr])
        self.w_b_l = [self.xfpr + self.w_b_l[0], self.yfpr + self.w_b_l[1], self.zfpr + self.w_b_l[2]]

        self.w_b_r = self.Body2Fixed(self.w_b_right,[self.e0pr,self.expr,self.eypr,self.ezpr])
        self.w_b_r = [self.xfpr + self.w_b_r[0], self.yfpr + self.w_b_r[1], self.zfpr + self.w_b_r[2]]

        self.wing_l = self.Body2Fixed([0,-6.,-1.],[self.e0pr,self.expr,self.eypr,self.ezpr])
        self.wing_l = [self.xfpr + self.wing_l[0], self.yfpr + self.wing_l[1], self.zfpr + self.wing_l[2]]

        self.wing_r = self.Body2Fixed([0,6.,-1.],[self.e0pr,self.expr,self.eypr,self.ezpr])
        self.wing_r = [self.xfpr + self.wing_r[0], self.yfpr + self.wing_r[1], self.zfpr + self.wing_r[2]]

        #updates sim timestep to match graphics time step
        self.dt = t_g


        #update and pass state values as array
        y0 = np.array([self.upr, self.vpr, self.wpr, self.ppr, self.qpr, 
                       self.rpr, self.xfpr, self.yfpr, self.zfpr, self.e0pr,
                       self.expr, self.eypr, self.ezpr])

        #call to RK4 method
        sol = self.rnkta4(13, self.ti, y0, self.dt, self.derivs)

        #update time with time step
        self.ti = self.ti + self.dt

        #uses RK4 solution to update current state values

        #state velocities
        self.upr = sol[0]
        self.vpr = sol[1]
        self.wpr = sol[2]

        #state rotation rates
        self.ppr = sol[3]
        self.qpr = sol[4]
        self.rpr = sol[5]

        #state positions
        self.xfpr = sol[6]
        self.yfpr = sol[7]
        self.zfpr = sol[8]

        #state quaternions
        self.e0pr, self.expr, self.eypr, self.ezpr = self.NormQuat2([sol[9],
                                                                    sol[10], 
                                                                    sol[11], 
                                                                    sol[12]])

        #state euler angles
        self.phi, self.theta, self.psi = self.Quat2Euler([self.e0pr, 
                                                          self.expr, 
                                                          self.eypr,
                                                          self.ezpr],
                                                          self.psiprev)
        self.heading = self.fix_heading(self.psi)

        #state angle of attack and sideslip angle
        self.alpha = atan(self.wpr/self.upr)
        self.beta = atan(self.vpr/self.upr)

        self.psiprev = self.psi

        #state airspeed
        self.V = np.sqrt((self.upr*self.upr) + (self.vpr*self.vpr) + 
                         (self.wpr*self.wpr))

        #earth fixed velocities
        self.earth_vel()
        self.gnd_V = sqrt(self.Vxf*self.Vxf + self.Vyf*self.Vyf)

        #state densitity, constant or updated with alititude
        if self.constant_density:
            self.rho = self.rho_ref
        else:
            h,z,t,p,d = self.statee(-self.zfpr)
            self.rho = d


        self.state = np.array([self.upr, self.vpr, self.wpr, self.ppr, self.qpr, self.rpr, self.xfpr, self.yfpr, self.zfpr, self.e0pr, self.expr, self.eypr, self.ezpr])

#-----------------------------------------------------------------------------#
#REAL TIME STATE UPDATE FUNCTIONS

    def mass(self, t):

        '''Updates aircraft mass properties based on fuel and payload loss.
        Inverts the moment of inertia matrix.

        Parameters
        -----------
        t: integer or float
            current time step
        '''

        #aircraft mass properties, including inverse of interia tensor
        self.M = self.W/self.g

        self.Ixx = self.Ixx
        self.Iyy = self.Iyy
        self.Izz = self.Izz
        self.Ixy = self.Ixy
        self.Ixz = self.Ixz
        self.Iyz = self.Iyz

        self.Iinv = np.linalg.inv([[self.Ixx, -self.Ixy, -self.Ixz], 
                                   [-self.Ixy, self.Iyy, -self.Iyz],
                                   [-self.Ixz, -self.Iyz, self.Izz]])

    def wind(self, t, pos = 0):

        '''Updates wind velocities of the simulator based on time and position.

        Parameters
        -----------
        t: integer or float
            current time step
        pos: array of floats
            current position array
        '''

        self.Vwxf = 0.
        self.Vwyf = 0.
        self.Vwzf = 0.

    def control(self, t):

        '''Updates aircraft control inputs.

        Parameters
        -----------
        t: integer or float
            current time step
        '''

        #throttle setting
        self.tau = self.tau

        #aileron input
        self.da = self.da0

        #elevator input
        self.de = self.de0

        #rudder input
        self.dr = self.dr0

    def thrust(self,t,f):

        '''Updates aircraft thrust values.

        Parameters
        -----------
        t: integer or float
            current time step
        '''

        self.Txb = self.tau*((self.rho/self.rho_ref)**self.a)*(self.T0 + 
                          self.T1*self.V + self.T2*self.V*self.V)
        self.Tyb = 0.
        self.Tzb = 0.
        self.xbp = 0.
        self.ybp = 0.
        self.zbp = self.zT0
        self.hxb = 0.
        self.hyb = 0.
        self.hzb = 0.

    def aero(self,t,f):

        '''Updates aerodynamic forces and moments at each timestep and RK4 call.
        Now includes physics model for landing.

        Parameters
        -----------
        t: integer or float
            current time step
        f: array
            array of current state values
        '''

        u, v, w, p, q, r, xf, yf, zf, e0, ex, ey, ez = f

        #current angle of attack and sideslip angle
        alpha = atan2(w,u)
        beta = atan2(v,u)

        #ratios to non-dimensionalize rotation rates
        nonc = self.cb/(2*self.V)
        nonb = self.bw/(2*self.V)

        #non-dimensional rotation rates
        pb = (p*nonb)
        qb = (q*nonc)
        rb = (r*nonb)

        if self.trimmed == False:
            #stall model blending parameters
            a01 = 19*(np.pi/180.)
            M1 = 13.5
            a02 = 25*(np.pi/180.)
            M2 = 150.
            M3 = 5.5

            sign = alpha/abs(alpha)
            sig1 = (1 + exp(-M1*(alpha-a01)) + exp(M1*(alpha+a01)))/((1 + exp(-M1*(alpha-a01)))*(1 + exp(M1*(alpha+a01))))
            sig2 = (1 + exp(-M2*(alpha-a02)) + exp(M2*(alpha+a02)))/((1 + exp(-M2*(alpha-a02)))*(1 + exp(M2*(alpha+a02))))
            sig3 = (1 + exp(-M3*(alpha-a01)) + exp(M3*(alpha+a01)))/((1 + exp(-M3*(alpha-a01)))*(1 + exp(M3*(alpha+a01))))
        else:
            sig1 = 0
            sig2 = 0
            sig3 = 0
            sign = 1

        self.CL = ((1-sig1)*(self.CL_ref + self.CLa*alpha + self.CLq*qb) + 
                  (1-sig2)*(self.CLde*(self.de - self.de_ref)) 
                  + sig1*(2*sign*sin(alpha)*sin(alpha)*cos(alpha)))

        #current coefficient of sideforce
        self.Cs = (self.CYb*beta + self.CYp*pb + self.CYr*rb +
                   self.CYda*self.da + self.CYdr*self.dr)

        self.CD = ((1-sig3)*(self.CD0 + self.CD1*self.CL + self.CD2*self.CL*self.CL + self.CD3*self.Cs*self.Cs + self.CDq*qb + self.CDde*(self.de - self.de_ref)) + 
                    sig3*((1.98/2)*sin(2*alpha - (pi/2)) + (1.98/2)))


        #current rolling moment coefficient
        self.Cl = (self.Clb*beta + self.Clp*pb + (self.Clr/self.CL_ref)*self.CL*rb +
                   self.Clda*self.da + self.Cldr*self.dr)

        #current pitching moment coefficient
        if self.zfpr > -2.25:
            #alters pitching moment when landing to eliminate rocking of aircraft
            self.Cm = 0.5*(self.Cm_ref + self.Cmde*(self.de - self.de_ref))
        else:
            #avoids division by zero errors
            try:
                self.Cm = (self.Cm_ref + (self.Cma/self.CLa)*(self.CL*(u/self.V) - (1-sig1)*self.CL_ref
                        + self.CD*(w/self.V)) + self.Cmq*qb + self.Cmde*(self.de - self.de_ref))
            except:
                self.Cm = 0.

        #current yawing moment coefficient
        self.Cn = ((self.Cnb/self.CYb)*(self.Cs*(u/self.V) - self.CD*(v/self.V))
                    + (self.Cnp/self.CL_ref)*self.CL*pb + self.Cnr*rb + self.Cnda*self.da
                    + self.Cndr*self.dr)

        #coefficent to dimensionalize forces and moments
        coeff = 0.5*self.rho*self.V*self.V*self.Sw

        #current aerodynamic forces and moments
        self.X = coeff*(self.CL*sin(alpha) - self.Cs*sin(beta) - self.CD*(u/self.V))

        self.Y = coeff*(self.Cs*cos(beta) - self.CD*(v/self.V))

        self.Z = coeff*(-self.CL*cos(alpha) - self.CD*(w/self.V))


        self.Ml = coeff*self.bw*self.Cl

        self.Mm = (self.zT0*self.Txb) + coeff*self.cb*self.Cm

        # MAIN LANDING CODE SECTION

        #damping moment for yaw, coefficients adjusted as required for realism
        damp_n = 8.0*self.W*self.rpr
        dmpn_max = 4.0

        # sets a max yaw damping moment
        if damp_n > dmpn_max*self.W:
            damp_n = dmpn_max*self.W
        elif damp_n < -dmpn_max*self.W:
            damp_n = -dmpn_max*self.W

        # landing condition for yaw moment to provide steering on the ground
        if self.zfpr > -2.5:
            self.Mn = -100.0*self.W*(self.dr/15.) - damp_n
        else:
            #normal aero yaw moment
            self.Mn = coeff*self.bw*self.Cn


        v_fixed = self.Body2Fixed([u, v, w], [e0, ex, ey, ez])

        #landing Height, dependent on aircraft model
        h_t = 1.25

        # force on each wheel, set to zero for trim if not landing
        if self.zfpr > -2.25:
            F_w = self.W*(1./3)
        else:
            F_w = 0.

        # vertical damping force on wheels, function of fixed vertical velocity
        damp_f = 1.0*F_w*v_fixed[2]

        #conditions to set max damping force
        if damp_f > self.W and self.zfpr > -2.25:
            damp_f = self.W
        elif damp_f < -self.W and self.zfpr > -2.25:
            damp_f = -self.W
        elif self.zfpr < -2.25:
            damp_f = 0.0

        #spring-dampener system for each wheel, forces set to zero if not landing
        if self.w_b_l[2] > -(h_t):
            self.F_w_b_l = self.Fixed2Body([0,0,-(F_w*(self.w_b_l[2] + h_t)) - damp_f,0], [e0, ex, ey, ez])
        else:
            self.F_w_b_l = [0.,0.,0.]

        if self.w_b_r[2] > -(h_t):
            self.F_w_b_r = self.Fixed2Body([0,0,-(F_w*(self.w_b_r[2] + h_t)) - damp_f,0], [e0, ex, ey, ez])
        else:
            self.F_w_b_r = [0.,0.,0.]

        if self.w_f[2] > -(h_t):
            self.F_w_f = self.Fixed2Body([0,0,-(F_w*(self.w_f[2] + h_t)) - damp_f,0], [e0, ex, ey, ez])
        else:
            self.F_w_f = [0.,0.,0.]


        #converts earth fixed landing forces to body fixed, currently not being used
        #in the full manner that it could be
        self.F_w_total_x = (self.F_w_b_l[0] + self.F_w_b_r[0] + self.F_w_f[0])
        self.F_w_total_y = (self.F_w_b_l[1] + self.F_w_b_r[1] + self.F_w_f[1])
        self.F_w_total_z = (self.F_w_b_l[2] + self.F_w_b_r[2] + self.F_w_f[2])

#        print('f-left', self.F_w_b_l[2])
#        print('f-right', self.F_w_b_r[2])
#        print('f-front', self.F_w_f[2])

        # friction ratios help determine which direction to apply friction force
        # could probably just use upr and vpr values...

        fric_ratio = (self.upr/self.V0)
        side_fric_ratio = (self.vpr/self.V0)

        # axial and lateral friction forces, friction coefficients are constant
        # or determined by brake being applied\

        if fric_ratio > 0.0:
            self.F_fric = -self.cf*F_w
        elif fric_ratio < 0.0:
            self.F_fric = self.cf*F_w
        else:
            self.F_fric = 0.0


        if side_fric_ratio > 0.0:
            self.F_side_fric = -1.5*F_w
        elif side_fric_ratio < 0.0:
            self.F_side_fric = 1.5*F_w
        else:
            self.F_side_fric = 0.0

        # coefficient to adjust landing moments, dependent on aircraft
        m_c = 0.5

        # pitch damping moment
        damp_m = 8.0*self.W*self.qpr
        dmp_max = 1.5

        #max pitch damping conditions

        if damp_m > dmp_max*self.W and self.zfpr > -2.75:
            damp_m = dmp_max*self.W
        elif damp_m < -dmp_max*self.W and self.zfpr > -2.75:
            damp_m = -dmp_max*self.W
        elif self.zfpr < -2.75:
            damp_m = 0.0
#        print('damp_m', damp_m)

        # pitch moments on landing, set to zero if not landing
        if self.w_b_l[2] > -(h_t):
            self.m_w_b_l = -m_c*self.F_w_b_l[2]*self.w_b_left[0] - damp_m
        else:
            self.m_w_b_l= 0.

        if self.w_b_r[2] > -(h_t):
            self.m_w_b_r = -m_c*self.F_w_b_r[2]*self.w_b_right[0] - damp_m
        else:
            self.m_w_b_r = 0.

        if self.w_f[2] > -(h_t):
            self.m_w_f = -2*m_c*self.F_w_f[2]*self.w_front[0] - 2*damp_m
        else:
            self.m_w_f  = 0.


#        print('m-left', self.m_w_b_l)
#        print('m-right', self.m_w_b_r)
#        print('m-front', self.m_w_f)
#        print('m-aero', self.Mm)

        # roll damping moment
        damp_l = 8.0*self.W*self.ppr
        dmpl_max = 1.3

        # max roll damping moment conditions
        if damp_l > dmpl_max*self.W and self.zfpr > -2.75:
            damp_l = dmpl_max*self.W
        elif damp_l < -dmpl_max*self.W and self.zfpr > -2.75:
            damp_l = -dmpl_max*self.W
        elif self.zfpr < -2.75:
            damp_l = 0.0

        # roll moments on landing
        self.l_w_b_l = 10*m_c*self.F_w_b_l[2]*self.w_b_left[1] - damp_l
        self.l_w_b_r = 10*m_c*self.F_w_b_r[2]*self.w_b_right[1] - damp_l
#        print('l-left', self.l_w_b_l)
#        print('l-right', self.l_w_b_r)


#-----------------------------------------------------------------------------#
#INTEGRATION FUNCTIONS

    def derivs(self, t0, y0):

        '''Differential equations of rigid-body motion using the quaternion
        formulation. Calculates the derivatives of aircraft motion that are
        used in the Runge-Kutta 4 formulation. Defined in Mechanics of Flight,
        Second Edition, by Warren F. Phillips, Eq. (11.11.1) - (11.11.4).

        Parameters
        -----------
        t0: integer or float
            current time step
        y0: array
            array of current state values

        Returns
        -------
        sol: array
            array of state derivatives
        '''

        #current state values
        u, v, w, p, q, r, xf, yf, zf, e0, ex, ey, ez = y0

        #current airspeed
        self.V = np.sqrt((u*u) + (v*v) + (w*w))

        # conditions if landing and aircraft is level with no contorl inputs
        # this assists in graphics and keeping aircraft from bouncing around 
        # too much
        if self.zfpr > -2.5 and abs(self.theta*(180/np.pi)) < 0.1 and abs(self.phi*(180/np.pi)) < 0.5 and (self.de >= 0. or self.V < 5.0):
            q = 0.
        if self.zfpr > -2.5 and abs(self.phi*(180/np.pi)) < 0.5:
            p = 0.
        if self.zfpr > -2.5 and self.dr == 0.:
            r = 0.
            self.v = 0.

        #function calls to state inputs, provides force, moment, and control
        #input values at current state
        self.mass(t0)
        self.wind(t0)
        self.control(t0)
        self.thrust(t0,y0)
        self.aero(t0,y0)

        #velocity derivatives
        udot = (self.g*2*(ex*ez - ey*e0) + (self.g/self.W)*(self.X + self.Txb + self.F_w_total_x + self.F_fric) 
                + r*v - q*w)
        vdot = (self.g*2*(ey*ez + ex*e0) + (self.g/self.W)*(self.Y + self.Tyb + self.F_side_fric) + 
                p*w - r*u)
        wdot = (self.g*(ez*ez + e0*e0 - ex*ex - ey*ey) + 
                (self.g/self.W)*(self.Z + self.Tzb + self.F_w_total_z) + q*u - p*v)

        #variables for rotation rate derivatives
        t1 = (-self.hzb*q + self.hyb*r + self.Ml + self.l_w_b_l + self.l_w_b_r + (self.Iyy - self.Izz)*q*r + 
                self.Iyz*(q*q - r*r) + self.Ixz*p*q - self.Ixy*p*r)

        t2 = (self.hzb*p - self.hxb*r + self.Mm + self.m_w_b_l + self.m_w_b_r + self.m_w_f + (self.Izz - self.Ixx)*p*r + 
              self.Ixz*(r*r - p*p) + self.Ixy*q*r - self.Iyz*p*q)

        t3 = (-self.hyb*p + self.hxb*q + self.Mn + (self.Ixx - self.Iyy)*p*q + 
              self.Ixy*(p*p - q*q) + self.Iyz*p*r - self.Ixz*q*r)

        #rotation rate derivatives
        pdot = self.Iinv[0,0]*t1 + self.Iinv[0,1]*t2 + self.Iinv[0,2]*t3
        qdot = self.Iinv[1,0]*t1 + self.Iinv[1,1]*t2 + self.Iinv[1,2]*t3
        rdot = self.Iinv[2,0]*t1 + self.Iinv[2,1]*t2 + self.Iinv[2,2]*t3

        #position derivatives
        xdot, ydot, zdot = self.Body2Fixed([u, v, w], [e0,
                                       ex, ey, ez])
        xdot = xdot + self.Vwxf
        ydot = ydot + self.Vwyf
        zdot = zdot + self.Vwzf

        #quaternion derivatives
        e0dot = 0.5*(-ex*p - ey*q - ez*r)
        exdot = 0.5*(e0*p - ez*q + ey*r)
        eydot = 0.5*(ez*p + e0*q - ex*r)
        ezdot = 0.5*(-ey*p + ex*q + e0*r)

        #array of all state derivatives
        sol = np.array([udot, vdot, wdot, pdot, qdot, rdot,
                        xdot, ydot, zdot, e0dot, exdot, eydot, ezdot])

        return sol

    def rnkta4(self,n,t0,y0,dt,f):

        '''Updates aerodynamic forces and moments at each timestep and RK4 call

        Parameters
        -----------
        t0: integer or float
            timestep initial condition
        y0: array
            array of current state conditions
        dt: interger or float
            timestep size
        f: function
            derivative function of values being integrated forward

        Returns
        -------
        y: array
            Next step of state values, predicted through Runge-Kutta formulation
        '''

        # pre-allocate arrays
        k1i = np.zeros(n)
        k2i = np.zeros(n)
        k3i = np.zeros(n)
        k4i = np.zeros(n)

        # all array operations for n # of variables
        # Runge Kutta 4 formulation
        k1i = f(t0, y0)
        yi = y0 + 0.5*k1i*dt

        k2i = f(t0 + 0.5*dt, yi)
        yi = y0 + 0.5*k2i*dt

        k3i = f(t0 + 0.5*dt, yi)
        yi = y0 + k3i*dt

        k4i = f(t0 + dt, yi)

        #final Runge-Kutta estimate
        y = y0 + (1/6)*(k1i + 2*k2i + 2*k3i + k4i)*dt

        return y

 #-----------------------------------------------------------------------------#
#Equilibrium State Functions

    def fix_heading(self,psi):
        '''keeps heading between 0 and 360 degrees
        '''

        if psi < 0:
            h = 2*pi + psi
        else:
            h = psi
        return h

    def earth_vel(self):
        '''earth fixed velocities
        '''

        CT = cos(self.theta)
        CPS = cos(self.psi)
        CPH = cos(self.phi)
        ST = sin(self.theta)
        SPS = sin(self.psi)
        SPH = sin(self.phi)
        self.Vxf = CT*CPS*self.upr + (SPH*ST*CPS - CPH*SPS)*self.vpr + (CPH*ST*CPS + SPH*SPS)*self.wpr
        self.Vyf = CT*SPS*self.upr + (SPH*ST*SPS + CPH*CPS)*self.vpr + (CPH*ST*SPS - SPH*CPS)*self.wpr
        self.Vzf = -ST*self.upr + SPH*CT*self.vpr + CPH*CT*self.wpr

    def update_vel(self,V):

        '''finds xb, yb, and zb components of airspeed

        Parameters
        -----------
        V: float
            current airspeed
        '''

        denom = sqrt(1-sin(self.alpha)*sin(self.alpha)*sin(self.beta)*sin(self.beta))

        self.uo = V*(cos(self.alpha)*cos(self.beta))/denom
        self.vo = V*(cos(self.alpha)*sin(self.beta))/denom
        self.wo = V*(sin(self.alpha)*cos(self.beta))/denom

    def theta_solve(self):

        '''solves for elevation angle. (My formulation)

        Returns
        -------
        theta: float
            elevation angle
        '''

        denom = sqrt(1-sin(self.alpha)*sin(self.alpha)*sin(self.beta)*sin(self.beta))
        I = (cos(self.alpha)*cos(self.beta))/denom
        J = (cos(self.alpha)*sin(self.beta))/denom
        K = (sin(self.alpha)*cos(self.beta))/denom
        X = J*sin(self.phi) + K*cos(self.phi)
        Sy = sin(self.gamma)
        num1 = sqrt(I*I*X*X - X*X*Sy*Sy + X*X*X*X)

        theta1 = asin((sin(self.gamma)*I + num1)/(I*I + X*X))
        theta2 = asin((sin(self.gamma)*I - num1)/(I*I + X*X))

        check1 = X*cos(theta1) - sin(theta1)*I + sin(self.gamma)
        check2 = X*cos(theta2) - sin(theta2)*I + sin(self.gamma)

        if check1 < 1e-12:
            return theta1
        elif check2 < 1e-12:
            return theta2
        else:
            return 0.0

    def theta_solve2(self):

        '''solves for elevation angle 

        Returns
        -------
        theta: float
            elevation angle
        '''

        denom1 = sqrt(1-sin(self.alpha)*sin(self.alpha)*sin(self.beta)*sin(self.beta))
        C = (cos(self.alpha)*cos(self.beta))/denom1
        B = (cos(self.alpha)*sin(self.beta))/denom1
        A = (sin(self.alpha)*cos(self.beta))/denom1
        Sy = sin(self.gamma)
        num = (B*sin(self.phi) + A*cos(self.phi))*sqrt(C*C + ((B*sin(self.phi) + A*cos(self.phi))**2) - Sy*Sy)
        denom2 = C*C + ((B*sin(self.phi) + A*cos(self.phi))**2)
        theta1 = asin((C*Sy + num)/denom2)
        theta2 = asin((C*Sy - num)/denom2)
        check1 = C*sin(theta1) - (B*sin(self.phi) + A*cos(self.phi))*cos(theta1)-Sy
        check2 = C*sin(theta2) - (B*sin(self.phi) + A*cos(self.phi))*cos(theta2)-Sy

        if check1 < 1e-12:
            return theta1
        elif check2 < 1e-12:
            return theta2
        else:
            return 0.0

    def trim_mat(self, CT, pb0, qb0, rb0):

        '''trim matrix from higher expanded solution (my solution)

        Parameters
        ----------
        CT: float
            coefficient of thrust
        pb0: float
            non-dimensional rolling rate
        qb0: float
            non-dimensional pitching
        rb0: float
            non-dimensional yawing rate

        Returns
        -------
        mat: 2D array
            trim LHS matrix
        '''

        mat = np.array([[CT, (sin(self.alpha) - (self.ur*self.CD1))*self.CLa, -sin(self.beta)*self.CYb, sin(self.alpha)*self.CLde - self.ur*(self.CD1*self.CLde + self.CDde), -sin(self.beta)*self.CYda, -sin(self.beta)*self.CYdr],
                        [0., -self.vr*self.CD1*self.CLa, cos(self.beta)*self.CYb, -self.vr*(self.CD1*self.CLde + self.CDde), cos(self.beta)*self.CYda, cos(self.beta)*self.CYdr],
                        [0., (-cos(self.alpha) - (self.wr*self.CD1))*self.CLa, 0., -cos(self.alpha)*self.CLde - self.wr*(self.CD1*self.CLde + self.CDde), 0, 0],
                        [0., (self.Clr/self.CL_ref)*rb0*self.CLa, self.Clb, (self.Clr/self.CL_ref)*rb0*self.CLde, self.Clda, self.Cldr],
                        [self.zT0*CT/self.cb, self.Cma*(self.ur + self.wr*self.CD1), 0., (self.Cma/self.CLa)*(self.ur*self.CLde + self.wr*self.CD1*self.CLde + self.wr*self.CDde) + self.Cmde, 0., 0.],
                        [0., self.CLa*((self.Cnp/self.CL_ref)*pb0 - (self.Cnb/self.CYb)*self.vr*self.CD1), self.ur*self.Cnb, -(self.Cnb/self.CYb)*self.vr*(self.CD1*self.CLde + self.CDde) + (self.Cnp/self.CL_ref)*pb0*self.CLde, (self.Cnb/self.CYb)*self.ur*self.CYda + self.Cnda, (self.Cnb/self.CYb)*self.ur*self.CYdr + self.Cndr]])
        return mat

    def trim_array(self, p0, q0, r0, pb0, qb0, rb0):

        '''trim array from higher expanded solution (my solution)

        Parameters
        ----------
        p0: float
            rolling rate
        q0: float
            pitching rate
        r0: float
            yawing rate
        p0: float
            non-dimensional rolling rate
        qb0: float
            non-dimensional pitching
        rb0: float
            non-dimensional yawing rate

        Returns
        -------
        array: array
            trim RHS array
        '''

        array = np.array([[-sin(self.alpha)*(self.CL_ref + self.CLq*qb0) + sin(self.beta)*(self.CYp*pb0 + self.CYr*rb0) + self.ur*self.X1 + self.Cw*sin(self.theta) - (self.Cw/self.g)*(r0*self.vo - q0*self.wo)],
                          [-cos(self.beta)*(self.CYp*pb0 + self.CYr*rb0) + self.vr*self.X1 - self.Cw*sin(self.phi)*cos(self.theta) - (self.Cw/self.g)*(p0*self.wo - r0*self.uo)],
                          [cos(self.alpha)*(self.CL_ref + self.CLq*qb0) + self.wr*self.X1 - self.Cw*cos(self.phi)*cos(self.theta) - (self.Cw/self.g)*(q0*self.uo - p0*self.vo)],
                          [-self.Clp*pb0 - (self.Clr/self.CL_ref)*rb0*(self.CL_ref + self.CLq*qb0) + (self.Cw/(self.W*self.bw))*(self.hzb*q0 - self.hyb*r0 - (self.Iyy - self.Izz)*q0*r0 - self.Iyz*(q0*q0 - r0*r0) - self.Ixz*p0*q0 + self.Ixy*p0*r0)],
                          [-self.Cm_ref - (self.Cma/self.CLa)*(self.ur*(self.CL_ref + self.CLq*qb0) - self.CL_ref + self.wr*self.X1) - self.Cmq*qb0 + (self.Cw/(self.W*self.cb))*(-self.hzb*p0 + self.hxb*r0 - (self.Izz - self.Ixx)*p0*r0 - self.Ixz*(r0*r0 - p0*p0) - self.Ixy*q0*r0 + self.Iyz*p0*q0)],
                          [-(self.Cnb/self.CYb)*self.ur*(self.CYp*pb0 + self.CYr*rb0) + (self.Cnb/self.CYb)*self.vr*self.X1 - (self.Cnp/self.CL_ref)*pb0*(self.CL_ref + self.CLq*qb0) - self.Cnr*rb0 + (self.Cw/(self.W*self.bw))*(self.hyb*p0 - self.hxb*q0 - (self.Ixx - self.Iyy)*p0*q0 - self.Ixy*(p0*p0 - q0*q0) - self.Iyz*p0*r0 + self.Ixz*q0*r0)]])
        return array

    def trim_mat2(self, CT, pb0, qb0, rb0):

        '''trim matrix from lower expanded solution (provided solution)

        Parameters
        ----------
        p0: float
            rolling rate
        q0: float
            pitching rate
        r0: float
            yawing rate
        p0: float
            non-dimensional rolling rate
        qb0: float
            non-dimensional pitching
        rb0: float
            non-dimensional yawing rate

        Returns
        -------
        array: array
            trim RHS array
        '''

        mat = np.array([[CT, 0., 0., -self.CDde*self.ur, 0., 0.],
                        [0., 0., cos(self.beta)*self.CYb, 0., cos(self.beta)*self.CYda, cos(self.beta)*self.CYdr],
                        [0., -cos(self.alpha)*self.CLa, 0., -cos(self.alpha)*self.CLde, 0, 0],
                        [0., 0., self.bw*self.Clb, 0., self.bw*self.Clda, self.bw*self.Cldr],
                        [self.zT0*CT, 0., 0., self.cb*self.Cmde, 0., 0.],
                        [0., 0., 0., 0., self.bw*self.Cnda, self.bw*self.Cndr]])
        return mat

    def trim_array2(self, p0, q0, r0, pb0, qb0, rb0):

        '''trim array from lower expanded solution (provided solution))

        Parameters
        ----------
        p0: float
            rolling rate
        q0: float
            pitching rate
        r0: float
            yawing rate
        p0: float
            non-dimensional rolling rate
        qb0: float
            non-dimensional pitching
        rb0: float
            non-dimensional yawing rate

        Returns
        -------
        array: array
            trim RHS array
        '''

        array = np.array([[-self.CL*sin(self.alpha) + self.Cs*sin(self.beta) + self.ur*(self.CD0 + self.CD1*self.CL + self.CD2*self.CL*self.CL + self.CD3*self.Cs*self.Cs + self.CDq*qb0) + self.Cw*sin(self.theta) - (self.Cw/self.g)*(r0*self.vo - q0*self.wo)],
                          [(-self.CYp*pb0 - self.CYr*rb0)*cos(self.beta) + self.CD*self.vr - self.Cw*sin(self.phi)*cos(self.theta) - (self.Cw/self.g)*(p0*self.wo - r0*self.uo)],
                          [(self.CL_ref + self.CLq*qb0)*cos(self.alpha) + self.CD*self.wr - self.Cw*cos(self.phi)*cos(self.theta) - (self.Cw/self.g)*(q0*self.uo - p0*self.vo)],
                          [-self.bw*(self.Clp*pb0 + (self.Clr/self.CL_ref)*self.CL*rb0) + (self.Cw/(self.W))*(self.hzb*q0 - self.hyb*r0 - (self.Iyy - self.Izz)*q0*r0 - self.Iyz*(q0*q0 - r0*r0) - self.Ixz*p0*q0 + self.Ixy*p0*r0)],
                          [-self.cb*(self.Cm_ref + (self.Cma/self.CLa)*(self.CL*self.ur - self.CL_ref + self.CD*self.wr) + self.Cmq*qb0) + (self.Cw/(self.W))*(-self.hzb*p0 + self.hxb*r0 - (self.Izz - self.Ixx)*p0*r0 - self.Ixz*(r0*r0 - p0*p0) - self.Ixy*q0*r0 + self.Iyz*p0*q0)],
                          [-self.bw*((self.Cnb/self.CYb)*(self.Cs*self.ur - self.CD*self.vr) + (self.Cnp/self.CL_ref)*self.CL*pb0 + self.Cnr*rb0) +(self.Cw/(self.W))*(self.hyb*p0 - self.hxb*q0 - (self.Ixx - self.Iyy)*p0*q0 - self.Ixy*(p0*p0 - q0*q0) - self.Iyz*p0*r0 + self.Ixz*q0*r0)]])
        return array

#-----------------------------------------------------------------------------#
#Quaternion Operation Functions

    def NormQuat(self, q):

        '''normalizes quaternion using Eq. (11.10.7) from Phillips

        Parameters
        ----------
        q: array
            quaternion values

        Returns
        -------
        list of normalized quaternion values
        '''

        q0, q1, q2, q3 = q

        norm_coeff = 1.5 - 0.5*(q0*q0 + q1*q1 + q2*q2 + q3*q3)

        return [q0*norm_coeff, q1*norm_coeff, q2*norm_coeff,
                q3*norm_coeff]

    def NormQuat2(self, q):

        '''normalizes quaternion using Eq. (11.10.5) from Phillips

        Parameters
        ----------
        q: array
            quaternion values

        Returns
        -------
        list of normalized quaternion values
        '''
        q0, q1, q2, q3 = q

        norm_den = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)

        return [q0/norm_den, q1/norm_den, q2/norm_den,
                q3/norm_den]

    def Euler2Quat(self, ea):

        '''converts euler angles to quaternion using Eq. (11.7.8)

        Parameters
        ----------
        ea: array
            euler angles

        Returns
        -------
        list of quaternion values
        '''

        ea05 = ea[0]*0.5
        ea15 = ea[1]*0.5
        ea25 = ea[2]*0.5

        CP = cos(ea05)
        CT = cos(ea15)
        CS = cos(ea25)

        SP = sin(ea05)
        ST = sin(ea15)
        SS = sin(ea25)
        c1 = CP*CT
        c2 = SP*ST
        c3 = SP*CT
        c4 = CP*ST

        return [(c1*CS + c2*SS), (c3*CS - c4*SS),
                (c4*CS + c3*SS), (c1*SS - c2*CS)]

    def Quat2Euler(self, q, psiprev=0):

        '''converts quaternion to euler angles using Eq. (11.7.11)

        Parameters
        ----------
        q: array
            quaternion values
        psiprev: float
            previous psi value

        Returns
        -------
        list of euler angles
        '''

        q0, q1, q2, q3 = q

        check = q0*q2 - q1*q2

        if (check) == 0.5:
            phi = 2*asin(q1/cos(pi/4)) + psiprev
            theta = pi/2
            psi = psiprev
            return [phi,theta,psi]
        elif (check) == -0.5:
            phi = 2*asin(q1/cos(pi/4)) - psiprev
            theta = -pi/2
            psi = psiprev
            return [phi,theta,psi]
        else:
            q00 = q0*q0
            q11 = q1*q1
            q22 = q2*q2
            q33 = q3*q3

            return [atan2(2*(q0*q1 + q2*q3),
                        (q00 + q33 - q11 - q22)),
                    asin(2*(q0*q2 - q1*q3)),
                    atan2(2*(q0*q3 + q1*q2),
                             (q00 + q11 - q22 - q33))]

    def Fixed2Body(self, v, e):

        '''converts earth fixed vector to body fixed vector using reduced form 
        of Eq. (11.6.8)

        Parameters
        ----------
        v: array
            vector to be converted
        e: array
            quaternion

        Returns
        -------
        list of body fixed vector components
        '''

        v0, v1, v2, v3 = v
        e0, e1, e2, e3 = e

        ve00 = v0*e0
        ve01 = v0*e1
        ve02 = v0*e2
        ve03 = v0*e3
        ve10 = v1*e0
        ve11 = v1*e1
        ve12 = v1*e2
        ve13 = v1*e3
        ve20 = v2*e0
        ve21 = v2*e1
        ve22 = v2*e2
        ve23 = v2*e3

        return [(e0*(ve00 + ve13 - ve22) - 
                      e1*(-ve01 - ve12 - ve23) -
                      e2*(ve02 - ve11 + ve20) +
                      e3*(-ve03 + ve10 + ve21)),
                    (e0*(-ve03 + ve10 + ve21) + 
                     e1*(ve02 - ve11 + ve20) -
                     e2*(-ve01 - ve12 - ve23) -
                     e3*(ve00 + ve13 - ve22)),
                    (e0*(ve02 - ve11 + ve20) - 
                     e1*(-ve03 + ve10 + ve21) +
                     e2*(ve00 + ve13 - ve22) -
                     e3*(-ve01 - ve12 - ve23))]

    def Body2Fixed(self, v, e):

        '''converts body fixed vector to earth fixed vector using reduced form 
        of Eq. (11.6.8) inversed

        Parameters
        ----------
        v: array
            vector to be converted
        e: array
            quaternion

        Returns
        -------
        list of earth fixed vector components
        '''

        e0, e1, e2, e3 = e
        v0, v1, v2 = v

        e00 = e0*e0
        e0x = e0*e1
        e0y = e0*e2
        e0z = e0*e3
        exx = e1*e1
        exy = e1*e2
        exz = e1*e3
        eyy = e2*e2
        eyz = e2*e3
        ezz = e3*e3
        exzv2 = exz*v2
        exyv0 = exy*v0
        e0zv0 = e0z*v0
        e0yv0 = e0y*v0
        exzv0 = exz*v0
        exyv1 = exy*v1
        e0zv1 = e0z*v1
        eyzv1 = eyz*v1
        e0xv1 = e0x*v1
        e0yv2 = e0y*v2
        eyzv2 = eyz*v2
        e0xv2 = e0x*v2

        return [(e00*v0 - e0zv1 + e0yv2 + 
                      exx*v0 + exyv1 + exzv2 +
                      eyy*-v0 + exyv1 + e0yv2 -
                      ezz*v0 - e0zv1 + exzv2),
                    (e0zv0 + e00*v1 - e0xv2 + 
                     exyv0 - exx*v1 - e0xv2 +
                     exyv0 + eyy*v1 + eyzv2 +
                     e0zv0 - ezz*v1 + eyzv2),
                    (-e0yv0 + e0xv1 + e00*v2 + 
                     exzv0 + e0xv1 - exx*v2 -
                     e0yv0 + eyzv1 - eyy*v2 +
                     exzv0 + eyzv1 + ezz*v2)]

#-----------------------------------------------------------------------------#
#Result plotting and writing functions

    def write_results(self, i):

        '''writes results to txt file

        Parameters
        ----------
        i: integer
            i= 0 is header, i = 1 writes values
        '''

        if i == 0:
            self.f.truncate(0)
            self.f.write('{0:<17} {1:<17} {2:<17} {3:<17} {4:<17} {5:<17} {6:<17} {7:<17} {8:<17} {9:<17} {10:<17} {11:<17} {12:<17} {13:<17}'.format('   Time[sec]', '   u[ft/s]', '   v[ft/s]', '   w[ft/s]', '   p[rad/s]', '   q[rad/s]', '   r[rad/s]', '   x[ft]', '   y[ft]', '   z[ft]', '   eo', '   e1', '   e2', '   e3')+'\n')
        elif i == 1:
            self.f.write('{0:>18.9E} {1:>17.9E} {2:>17.9E} {3:>17.9E} {4:>17.9E} {5:>17.9E} {6:>17.9E} {7:>17.9E} {8:>17.9E} {9:>17.9E} {10:>17.9E} {11:>17.9E} {12:>17.9E} {13:>17.9E}'.format(self.ti, self.upr, self.vpr, self.wpr, self.ppr, self.qpr, self.rpr, self.xfpr, self.yfpr, self.zfpr, self.e0pr, self.expr,self.eypr, self.ezpr)+'\n')

    def write_trim(self):

        '''writes trim results to .txt file
        '''

        rnum = 12

        with(open('trim_results.txt', 'w+', encoding='utf-8')) as f:
            f.write('{0:<60}'.format("Number of iterations to convergence = " + str(self.count)) +'\n')
            f.write('{0:<60}'.format("cw = " + str(round(self.cb, rnum))) +'\n')
            f.write('{0:<60}'.format("CLref = " + str(round(self.CL_ref, rnum))) +'\n')
            f.write('{0:<60}'.format("CDO = " + str(round(self.CD0, rnum))) +'\n')
            f.write('{0:<60}'.format("CD1 = " + str(round(self.CD1, rnum))) +'\n')
            f.write('{0:<60}'.format("CD2 = " + str(round(self.CD2, rnum))) +'\n')
            f.write('{0:<60}'.format("Cmref = " + str(round(self.Cm_ref, rnum))) +'\n')
            f.write('{0:<60}'.format("Throttle = " + str(round(self.tau, rnum))) +'\n')
            f.write('{0:<60}'.format("alpha = " + str(round(self.alpha*(180/pi), rnum))) +'\n')
            f.write('{0:<60}'.format("beta = " + str(round(self.beta*(180/pi), rnum))) +'\n')
            f.write('{0:<60}'.format("de = " + str(round(self.de0*(180/pi), rnum))) +'\n')
            f.write('{0:<60}'.format("da = " + str(round(self.da0*(180/pi), rnum))) +'\n')
            f.write('{0:<60}'.format("dr = " + str(round(self.dr0*(180/pi), rnum))) +'\n')
            f.write('{0:<60}'.format("Theta0 = " + str(round(self.theta*(180/pi), rnum))))
        f.close()

    def print_trim(self):

        '''prints trim results to console
        '''
        rnum = 12
        print("Trim")
        print("Number of iterations to convergence = ", self.count)
        print("cw = ", round(self.cb, rnum))
        print("CLref = ", round(self.CL_ref, rnum))
        print("CDO = ", round(self.CD0, rnum))
        print("CD1 = ", round(self.CD1, rnum))
        print("CD2 = ", round(self.CD2, rnum))
        print("Cmref = ", round(self.Cm_ref, rnum))
        print("Throttle = ", round(self.tau, rnum))
        print("alpha = ", round(self.alpha*(180/pi), rnum))
        print("beta = ", round(self.beta*(180/pi), rnum))
        print("de = ", round(self.de0*(180/pi), rnum))
        print("da = ", round(self.da0*(180/pi), rnum))
        print("dr = ", round(self.dr0*(180/pi), rnum))
        print("Theta0 = ", round(self.theta*(180/pi), rnum))

    def plot_results(self, i):

        '''plots simulator results

        Parameters
        ----------
        i: integer
            i= 0 initializes lists, i = 1 appends lists, i = 2 plots results
        '''

        #appends and stores values for plotting, i = 2 then plots results
        if i == 0:
            self.time_plot = []
            self.range_plot = []
            self.airspeed_plot = []
            self.x_plot = []
            self.y_plot = []
            self.z_plot = []
            self.phi_plot = []
            self.theta_plot = []
            self.alpha_plot = []
            self.beta_plot = []
        elif i == 1:
            self.time_plot.append(self.ti)
#            self.range_plot.append(self.range_check)
            self.airspeed_plot.append(self.V)
            self.x_plot.append(self.xfpr)
            self.y_plot.append(self.yfpr)
            self.z_plot.append(-self.zfpr)
            self.phi_plot.append(self.phi*(180/pi))
            self.theta_plot.append(self.theta*(180/pi))
            self.alpha_plot.append(self.alpha*(180/pi))
            self.beta_plot.append(self.beta*(180/pi))
        elif i == 2:
            
#            fig, ax1 = plt.subplots()
#            ax2 = ax1.twinx()
#            ax1.plot(self.time_plot, self.alpha_plot, 'g-')
#            ax2.plot(self.time_plot, self.airspeed_plot, 'b-')

#            ax1.set_xlabel('Time, s')
#            ax1.set_ylabel('Angle of Attack (degrees)')
#            ax2.set_ylabel('Airspeed (degrees)')
#
#            plt.show()

            plt.figure(1)
            plt.plot(self.time_plot, self.theta_plot)
            plt.xlabel('Time, s')
            plt.ylabel('Elevation Angle (degrees)')
#            plt.xlim([0,100])

            plt.show()
            plt.figure(2)
            plt.plot(self.time_plot, self.phi_plot)
            plt.xlabel('Time, s')
            plt.ylabel('Bank Angle (degrees)')
#            plt.xlim([0,100])
            plt.show()
#
#            plt.plot(self.range_plot, self.alpha_plot)
#            plt.plot(self.range_plot, self.beta_plot)
#            plt.xlabel('Range (yd)')
#            plt.ylabel('Aerodynamic Angles (degrees)')
#            plt.legend(['Alpha', 'Beta'])
##            plt.xlim([0,100])
#            plt.show()
#
#            plt.plot(self.y_plot, self.x_plot)
#            plt.xlabel('y distance (yd)')
#            plt.ylabel('x distance (yd)')
##            plt.xlim([-100, 100])
##            plt.ylim([-100,100])
#            plt.show()
            plt.figure(3)
            plt.plot(self.time_plot, self.z_plot)
            plt.xlabel('Time, s')
            plt.ylabel('Height (yd)')
#            plt.xlim([0,100])
            plt.show()
            plt.figure(4)
            plt.plot(self.time_plot, self.alpha_plot)
            plt.xlabel('Time, s')
            plt.ylabel('Angle of Attack (degrees)')
#            plt.xlim([0,100])
            plt.show()
            plt.figure(5)
            plt.plot(self.time_plot, self.airspeed_plot)
            plt.xlabel('Time, s')
            plt.ylabel('Airspeed (ft/s)')
#            plt.xlim([0,100])
            plt.show()

            fig = plt.figure(figsize = (10,10))
            ax = fig.add_subplot(111, projection = '3d')
            ax.plot(self.y_plot, self.x_plot, self.z_plot)
            plt.show()

#-----------------------------------------------------------------------------#
#ATMOSPHERIC PROFILE FUNCTIONS

    def statsi(self, h):
        Psa = np.zeros(9)
        zsa = [0,11000,20000,32000,47000,52000,61000,79000,9.9e20]
        Tsa = [288.15,216.65,216.65,228.65,270.65,270.65,252.65,180.65,180.65]
        g0 = 9.80665
        R = 287.0528
        Re = 6356766
        Psa[0] = 101325
        z = Re*h/(Re+h)
        for i in range(1,9):
            Lt = -(Tsa[i]-Tsa[i-1])/(zsa[i]-zsa[i-1])
            if Lt == 0:
                if z <= zsa[i]:
                    t = Tsa[i-1]
                    p = Psa[i-1]*exp(-g0*(z-zsa[i-1])/R/Tsa[i-1])
                    d = (p/R)/t
                    return (h,z,t,p,d)
                else:
                    Psa[i] = Psa[i-1]*exp(-g0*(zsa[i]-zsa[i-1])/R/Tsa[i-1])
            else:
                ex = (g0/R)/Lt
                if z < zsa[i]:
                   t = Tsa[i-1]-Lt*(z-zsa[i-1])
                   p = Psa[i-1]*(t/Tsa[i-1])**ex
                   d = (p/R)/t
                   return (h,z,t,p,d)
                else:
                   Psa[i] = Psa[i-1]*(Tsa[i]/Tsa[i-1])**ex
        t = Tsa[8]
        p = 0.
        d = 0.
        return (h,z,t,p,d)

    def statee(self, h):
        #     h = geometric altitude, specified by user (ft)
        #     z = geopotential altitude, returned by subroutine (ft)
        #     t = temperature, returned by subroutine (R)
        #     p = pressure, returned by subroutine (lbf/ft**2)
        #     d = density, returned by subroutine (slugs/ft**3)
        hsi = h*0.3048
        hsi,zsi,tsi,psi,dsi = self.statsi(hsi)
        z = zsi/0.3048
        t = tsi*1.8
        p = psi*0.02088543
        d = dsi*0.001940320
        return (h,z,t,p,d)
