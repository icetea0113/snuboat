import numpy as np

class ShipDyn():
    def __init__(self):
        # Principal dimensions
        self.L = 1.8
        self.B = 0.56
        self.M = 42.0
        self.xp = 0.5
        self.yp = 0.2
        self.AT = 0.5
        self.AL = 1.0
        self.kzz = 0.4*self.L
        self.Izz = self.kzz**2*self.M

        # Thruster parameters
        self.rps_max = 30
        self.rps_min = -30
        self.del_rate = 15
        self.del_max = 30
        self.del_min = -30
        self.rps_dead = 10

        # self.uuu = np.array([-173.135,	-219.96280,	511.24930,	1995,	-12961,	2077,	15.8382,	12.2293])
        # self.vvv = np.array([-837.57290,	346.78180,	-324.60110,	168.436,	1432.80,	12165,	24.9230,	455.010])
        # self.rrr = np.array([478.14530,	-143.98910,	428.27620,	250.39920,	-5655.10,	16659,	-126.510,	309.940])
        # # self.uuuF = self.uuu[6] + self.uuu[8]*0.5
        # # self.uuuR = self.uuu[7] + self.uuu[0]*0.5
        # self.uuuF = 15.8382
        # self.uuuR = 12.2293

        self.uuu = np.array([-189.741637761389,
                    54.3238098991888,
                    167.742915447050,
                    2076.91913081928,
                    -15842.2826594921,
                    -756.372331277787,
                    1.55476967805240,
                    1.04058366711357,
                    1.85290225608397,
                    0.893041886826208])
        self.vvv = np.array([-950.307034213021,
                    818.674184147866,
                    -3054.26126138803,
                    -1589.07872109637,
                    1183.20000000000,
                    9661.28841581641,
                    54001.2312394495,
                    71332.1753230588])
        self.rrr = np.array([540.219996966093,
                    -303.614582429747,
                    1244.92755651894,
                    188.831736039731,
                    -6530.85383419923,
                    23075.4936791729,
                    -36088.4517668859,
                    20781.4910362461])

        self.uuuF = (self.uuu[6]+self.uuu[8])*0.5
        self.uuuR = (self.uuu[7]+self.uuu[9])*0.5

      
    def update_ship_state(self, pos, vel, ctrl_input, ctrl, wind_state, dt):
        x, y, psi = pos
        u, v, r = vel
        delP_cmd, delS_cmd, rpsP_cmd, rpsS_cmd = ctrl_input
        delP, delS, rpsP, rpsS = ctrl
        WS, WD = wind_state
        
        # update steering
        if abs(delP_cmd-delP) < self.del_rate*dt:
            delP_new = delP_cmd
        else:
            delP_new = np.sign(delP_cmd-delP)*self.del_rate*dt + delP
        if abs(delS_cmd-delS) < self.del_rate*dt:
            delS_new = delS_cmd
        else:
            delS_new = np.sign(delS_cmd-delS)*self.del_rate*dt + delS
        delP_new = np.clip(delP_new, self.del_min, self.del_max)
        delS_new = np.clip(delS_new, self.del_min, self.del_max)
        
        delPR_new = np.deg2rad(delP_new)
        delSR_new = np.deg2rad(delS_new)
        
        
        # update rps
        rpsP_new = rpsP_cmd
        rpsS_new = rpsS_cmd
        
        # rpsP_new = 0.0 if abs(rpsP_new) < 5.0 else rpsP_new
        # rpsS_new = 0.0 if abs(rpsS_new) < 5.0 else rpsS_new
        
        # cal wind force
        CX = -0.53*np.cos(-WD-np.pi+psi)
        CY = 0.90*np.sin(-WD-np.pi+psi)
        CN = 0.30*np.sin(2*(-WD-np.pi+psi))
        
        WX = CX*(0.5*1.225*WS**2*self.AT)
        WY = CY*(0.5*1.225*WS**2*self.AL)
        WN = CN*(0.5*1.225*WS**2*self.L*self.AL)

        # cal hull force
        if(abs(u < 0.5)):
            XH = np.dot(self.uuu[:6], np.array([u, v**2, u*v**2, v**4, r**2, v*r]))/(4**3)
            YH = np.dot(self.vvv, np.array([v, u*v, v*abs(v), u*v*abs(v), r, r*abs(r), v**2*r, v*r**2]))/(4**3)
            NH = np.dot(self.rrr, np.array([v, u*v, v*abs(v), u*v*abs(v), r, r*abs(r), v**2*r, v*r**2]))/(4**3)
            
            # cal PR force
            if np.sign(rpsP_new) >= 0:
                TP = self.uuuF*rpsP_new*abs(rpsP_new)/(4**3)
            else:
                TP = self.uuuR*rpsP_new*abs(rpsP_new)/(4**3)

            if np.sign(rpsS_new) >= 0:
                TS = self.uuuF*rpsS_new*abs(rpsS_new)/(4**3)
            else:
                TS = self.uuuR*rpsS_new*abs(rpsS_new)/(4**3)

            XP = TP*np.cos(delPR_new) + TS*np.cos(delSR_new)
            YP = -TP*np.sin(delPR_new) + -TS*np.sin(delSR_new)
            NP = -self.xp*YP + self.yp*(TP*np.cos(delPR_new)-TS*np.cos(delSR_new))

            # cal acc
            udot_new = (XH+XP+WX+v*r)/self.M
            vdot_new = (YH+YP+WY-u*r)/self.M
            rdot_new = (NH+NP+WN)/self.Izz
        else:
            rpm = rpsP_new*60
            Mu = (-5.8590E-03)*abs(delP_new)+(8.6759E-05)*abs(rpm)+3
            Mv = 3.4616E-01
            Mr = np.clip((7.0359E-03)*abs(delP_new)+(1.5531E-04)*abs(rpm)+9.5371E-01, 0.4, 2)
            u_des = ((-3.6136E-05)*rpm*abs(delP_new) + (2.4715E-03)*rpm)/2
            v_des = ((2.2218E-05)*rpm*delP_new + (1.8195E-02)*delP_new)/2
            r_des = ((4.63E-11)*rpm**2*delP_new*abs(delP_new) + (-2.50E-7)*rpm*delP_new*abs(delP_new) + (5.20E-4)*delP_new*abs(delP_new) + (-7.59E-11)*rpm**2*delP_new + (3.69E-6)*rpm*delP_new + (-4.36E-4)*delP_new)

            udot_new = (-u + u_des + WX/1000)/Mu
            vdot_new = (-v + v_des + WY/1000)/Mv
            rdot_new = (-r + r_des + WN/1000)/Mr
            
        # cal vel
        u_new = u + udot_new*dt
        v_new = v + vdot_new*dt
        r_new = r + rdot_new*dt
        
        # cal pos
        psi_new = psi + r*dt
        # psi_new = self.CheckHeadingBound(-np.pi, np.pi, psi_new)
        psi_new = np.mod(psi_new, 2*np.pi)
        if psi_new > np.pi:
            psi_new -= 2*np.pi

        xdot_new = np.cos(psi)*u - np.sin(psi)*v
        ydot_new = np.sin(psi)*u + np.cos(psi)*v

        x_new = x + xdot_new*dt
        y_new = y + ydot_new*dt
        
        pos_new = np.array([x_new, y_new, psi_new])
        vel_new = np.array([u_new, v_new, r_new])
        ctrl_new = np.array([delP_new, delS_new, rpsP_new, rpsS_new])
        
        return pos_new, vel_new, ctrl_new