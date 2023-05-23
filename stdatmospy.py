# Troy Abraham
# MAE 6510
# Problem 1


import math
import numpy as np

def statee(h):
    #     h = geometric altitude, specified by user (ft)
    #     z = geopotential altitude, returned by subroutine (ft)
    #     t = temperature, returned by subroutine (R)
    #     p = pressure, returned by subroutine (lbf/ft**2)
    #     d = density, returned by subroutine (slugs/ft**3)
    hsi = h*0.3048
    hsi,zsi,tsi,psi,dsi = statsi(hsi)
    z = zsi/0.3048
    t = tsi*1.8
    p = psi*0.02088543
    d = dsi*0.001940320
    return (h,z,t,p,d)

def statsi(h):
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
                p = Psa[i-1]*math.exp(-g0*(z-zsa[i-1])/R/Tsa[i-1])
                d = (p/R)/t
                return (h,z,t,p,d)
            else:
                Psa[i] = Psa[i-1]*math.exp(-g0*(zsa[i]-zsa[i-1])/R/Tsa[i-1])
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

def atm_print():

    with open('stdatmos_si.txt', 'w+') as f:
        f.truncate(0) # clears the file by truncating contents to the 0 position
        f.write('{}'.format(' Geometric Geopotential                                          Speed of')+"\n")
        f.write('{}'.format(' Altitude    Altitude   Temperature   Pressure      Density       Sound')+"\n")
        f.write('{}'.format('   (m)         (m)          (K)       (N/m**2)     (kg/m**3)      (m/s)')+"\n")
        f.write('{}'.format('--------------------------------------------------------------------------')+"\n")

        for i in range(0,50):
            ht = i*2000.
            h,z,t,p,d = statsi(ht)
            a = math.sqrt(1.4*287.0528*t)
            f.write('{:^7} {:^12d} {:^14.3f} {:^14.4E} {:^14.4E} {:^11.2f}'.format(int(h), int(z), t, p, d, a) +"\n")
        f.close()

    with open('stdatmos_ee.txt', 'w+') as f:
        f.truncate(0) # clears the file by truncating contents to the 0 position
        f.write('{}'.format(' Geometric Geopotential                                          Speed of')+"\n")
        f.write('{}'.format(' Altitude    Altitude   Temperature   Pressure      Density       Sound')+"\n")
        f.write('{}'.format('   (ft)        (ft)         (R)     (lbf/ft**2)  (slugs/ft**3)    (ft/s)')+"\n")
        f.write('{}'.format('--------------------------------------------------------------------------')+"\n")
    
        for i in range(0,50):
            ht = i*5000.
            h,z,t,p,d = statee(ht)
            tsi = t/1.8
            asi = math.sqrt(1.4*287.0528*tsi)
            a = asi/0.3048
            f.write('{:^7} {:^12d} {:^14.3f} {:^14.4E} {:^14.4E} {:^11.2f}'.format(int(h), int(z), t, p, d, a) +"\n")
        f.close()

