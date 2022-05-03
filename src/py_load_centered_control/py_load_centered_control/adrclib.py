from ast import IsNot
import numpy as np

class ADRC():
    def __init__(self, 
                 order: int, 
                 b0: float,
                 s_cl: float, 
                 k_eso: float, 
                 hz: float, 
                 init_state: tuple = False,
                 saturation: tuple = (False,False)):

        self.b = b0
        self.A = np.eye(order+1, k=1)
        self.B = np.zeros((order+1,1))
        self.B[-2] = b0
        self.C = np.zeros((1,order+1))
        self.C[0,0] = 1


        self.saturation = np.array(saturation,np.float64)

        self.u = 0.0
        if not(init_state):
            self.z = np.zeros((order+1,1))
        else:
            self.z = np.array(init_state,np.float64).reshape(-1,1)
        # Discretize
        self.Ad = np.eye(order+1)
        self.Bd = np.empty(self.B.size)
        for i in range(1,order+1):
            self.Ad = self.Ad + (np.linalg.matrix_power(self.A,i)*hz**-i)/np.math.factorial(i)
            self.Bd = self.Bd + (np.linalg.matrix_power(self.A,i-1)*hz**-i)/np.math.factorial(i)
        self.Bd = self.Bd@self.B

        z_eso = np.exp(s_cl*k_eso*1/hz)
        if order == 1:
            self.L = np.array([[1-z_eso**2], 
                                [hz*(1-z_eso)**2]])
        elif order == 2:
            self.L = np.array([[1-(z_eso)**3],
                                [(3*hz/2)*(1-z_eso)**2*(1+z_eso)],
                                [hz**2*(1-z_eso)**3]])
        # TODO: Calculate L for arbitrary order

        # Calculate gains for arbitrary order, gives [Kp Kd ... 1]
        # NOTE: Having 1 at end is utilized to simplify calculation in control function
        # as this is essentially the "gain" for the estimated disturbance
        self.gains = np.flip([np.poly(s_cl*np.ones(order))])


    def estimate(self, measurement):
        self.z = (self.Ad - self.L @ self.C @ self.Ad) @ self.z \
                    + (self.Bd - self.L @ self.C @ self.Bd) * self.u \
                    + self.L * measurement

    def control(self, ref):
        error = -self.z
        error[0] = ref + error[0]
        
        self.u = self.gains @ error / self.b
        if any(self.saturation):
            self.saturate()


    def saturate(self):
        self.u = np.clip(self.u,self.saturation[0],self.saturation[1])
