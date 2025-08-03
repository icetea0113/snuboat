import numpy as np 

class ShipDyn():
    def __init__(self):
        # Principal dimensions
        self.L = 9.0
        self.B = 3.0
        self.M = 4000.0
        self.xp = 2.0
        self.yp = 0.5
        self.AT = 4.0
        self.AL = 10.0
        self.kzz = 0.25*self.L
        self.Izz = self.kzz**2*self.M

        # Thruster parameters
        self.rps_max = 30
        self.rps_min = -30
        self.del_rate = 15
        self.del_max = 30
        self.del_min = -30
        self.thr_rate = 100
        self.thr_max = 30
        self.thr_min = -30
        self.rps_dead = 10

        # Control parameters
        self.Kp_u = -1000
        self.Kp_psi = -3000
        self.Kd_psi = -21000

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

        # Wind parameters
        self.f_init = 0.0001
        self.f_end = 10
        self.fs1 = np.linspace(self.f_init, 1, 3000)
        self.fs2 = np.linspace(1, 10, 1000)
        self.fs = np.hstack([self.fs1, self.fs2])
        self.n = len(self.fs)
        self.df = np.diff(self.fs)
        self.df = np.hstack([self.df, self.df[-1]])
        self.z = 0.773
        self.zs = 20.0
        self.I = 0.15*(self.z/self.zs)**(-0.125)
        self.alpha = 0.01
        # self.alpha = 0.005
        
        # joystick parameters
        self.thr_x_gain_dp = 0.3
        self.thr_y_gain_dp = 0.2
        self.thr_z_gain_dp = 0.1
        self.thr_gain_dp_bias = 5
        self.js_y_gear_start_point = 10
        self.js_x_gear_start_point = 10
        self.js_z_gear_start_point = 10
        self.js_y_thr_start_point = 40
        self.js_x_thr_start_point = 15
        self.js_z_thr_start_point = 30
        self.ratio_fwd_rvs = 1.65
        self.prop_low_KPfwd = 16.5
        self.prop_low_KPrvs = 10.0
        self.alloc_js_z_fb_thr = 0.25
        self.alloc_js_z_fb_del = 0.25
        self.alloc_r_fb = 100
        self.idle_rpm = 600
        self.thr_rpm_ratio = 35.7
        self.turning_u_gain = 50.0
        self.js_x_max = 80
        self.js_y_max = 60
        self.js_z_max = 15
        
        # self.thr_max_dock = 100
        self.thr_max_dock = 30
        # thr_max_fwd = self.equiv_fwd_thr(ctrl, thr_max_dock, thr_gain_dp_bias)
        self.thr_max_fwd = (1/self.ratio_fwd_rvs)*self.thr_max_dock + self.idle_rpm*(self.prop_low_KPrvs-self.prop_low_KPfwd)/(self.prop_low_KPrvs*self.thr_rpm_ratio) - self.thr_gain_dp_bias
        
    def update_ship_state(self, pos, vel, acc, ctrl_input, ctrl, wind_state, dt):
        x, y, psi = pos
        u, v, r = vel
        udot, vdot, rdot = acc
        delP_cmd, delS_cmd, thrP_cmd, thrS_cmd = ctrl_input
        delP, delS, thrP, thrS = ctrl
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
        
        # update throttle
        if abs(thrP_cmd-thrP) < self.thr_rate*dt:
            thrP_new = thrP_cmd
        else:
            thrP_new = np.sign(thrP_cmd-thrP)*self.thr_rate*dt + thrP
        if abs(thrS_cmd-thrS) < self.thr_rate*dt:
            thrS_new = thrS_cmd
        else:
            thrS_new = np.sign(thrS_cmd-thrS)*self.thr_rate*dt + thrS
            
        thrP_new = np.clip(thrP_new, self.thr_min, self.thr_max)
        thrS_new = np.clip(thrS_new, self.thr_min, self.thr_max)
        
        # update rps
        rpsP_new = 10.0*np.sign(thrP_new)+thrP_new
        rpsS_new = 10.0*np.sign(thrS_new)+thrS_new
        
        rpsP_new = 0.0 if abs(rpsP_new) < 11.0 else rpsP_new
        rpsS_new = 0.0 if abs(rpsS_new) < 11.0 else rpsS_new
        
        # cal wind force
        CX = -0.53*np.cos(-WD-np.pi+psi)
        CY = 0.90*np.sin(-WD-np.pi+psi)
        CN = 0.30*np.sin(2*(-WD-np.pi+psi))
        
        WX = CX*(0.5*1.225*WS**2*self.AT)
        WY = CY*(0.5*1.225*WS**2*self.AL)
        WN = CN*(0.5*1.225*WS**2*self.L*self.AL)

        # cal hull force
        XH = np.dot(self.uuu[:6], np.array([u, v**2, u*v**2, v**4, r**2, v*r]))
        YH = np.dot(self.vvv, np.array([v, u*v, v*abs(v), u*v*abs(v), r, r*abs(r), v**2*r, v*r**2]))
        NH = np.dot(self.rrr, np.array([v, u*v, v*abs(v), u*v*abs(v), r, r*abs(r), v**2*r, v*r**2]))
        
        # cal PR force
        # if np.sign(rpsP_new) >= 0:
        #     TP = self.uuuF*rpsP_new
        # else:
        #     TP = self.uuuR*rpsP_new

        # if np.sign(rpsS_new) >= 0:
        #     TS = self.uuuF*rpsS_new
        # else:
        #     TS = self.uuuR*rpsS_new

        if np.sign(rpsP_new) >= 0:
            TP = self.uuuF*rpsP_new*abs(rpsP_new)
        else:
            TP = self.uuuR*rpsP_new*abs(rpsP_new)

        if np.sign(rpsS_new) >= 0:
            TS = self.uuuF*rpsS_new*abs(rpsS_new)
        else:
            TS = self.uuuR*rpsS_new*abs(rpsS_new)

        XP = TP*np.cos(delPR_new) + TS*np.cos(delSR_new)
        YP = -TP*np.sin(delPR_new) + -TS*np.sin(delSR_new)
        NP = -self.xp*YP + self.yp*(TP*np.cos(delPR_new)-TS*np.cos(delSR_new))

        # cal acc
        udot_new = (XH+XP+WX+v*r)/self.M
        vdot_new = (YH+YP+WY-u*r)/self.M
        rdot_new = (NH+NP+WN)/self.Izz
        
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
        acc_new = np.array([udot_new, vdot_new, rdot_new])
        ctrl_new = np.array([delP_new, delS_new, thrP_new, thrS_new])
        hull_new = np.array([XH, YH, NH])
        prop_new = np.array([XP, YP, NP])
        wind_new = np.array([WX, WY, WN])
        thrust_new = np.array([TP, TS])
        
        return pos_new, vel_new, acc_new, ctrl_new, hull_new, prop_new, wind_new, thrust_new