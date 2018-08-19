
from math import sqrt, sin, cos, asin, atan2, pi




## EMA 필터 클래스
class FilterEMA(object) :
    def __init__(self, tabsize = 5, alpha = 0.5, use_lpf = False, lpf_factor = 0.1) :
        self.tab = [0] * tabsize
        self.tabsize = tabsize
        ## 속도를 위해 alpha 값 미리 계산해둠
        self.alpha = [1] * tabsize
        self.alpha[0] = alpha
        for ii in range(1,tabsize) :
            self.alpha[ii] = alpha*((1-alpha)**(ii) )
         
        self.useLpf = use_lpf
        self.lpfFactor = lpf_factor
        self.lpfTab = [0] * (tabsize + 1 )
        
    def put(self, value, get_result = False) :
        self.tab = [value] + self.tab[:-1]
        self.lpfTab = [value] + self.lpfTab[:-1]
        
        if get_result :
            return self.get()
        
    def get(self, bypass_lpf = False) :
        ## EMA_now = alpha * (val1 + ((1-alpha)**1)val2 + ((1-alpha)**2)val3 + .... )
        lpfValue = 0
        if len(self.tab) <= 0 :
            raise ValueError
        result = type(self.tab[0] )()
        for ii in range(len(self.tab) ) :
            result += self.tab[ii] * self.alpha[ii]
            if self.useLpf == True and bypass_lpf == False:
                lpfValue += self.alpha[ii] * ( (self.lpfFactor * self.tab[ii] )  \
                            + ( (1 - self.lpfFactor) * self.lpfTab[ii+1] ) )

        if self.useLpf == True and bypass_lpf == False:
            return result - lpfValue
        return result
            
class KalmanFilterGyroAccel(object) :
    def __init__(self, Q_angle = 0.02, Q_gyro = 0.0015, R_angle = 0.005) :
        self.Q_angle =  Q_angle
        self.Q_gyro = Q_gyro
        self.R_angle = R_angle
        self.P_00 = 0.0
        self.P_01 = 0.0
        self.P_10 = 0.0
        self.P_11 = 0.0
        self.bias = 0.0
        self.KFangle = 0.0
        
    def filter(self, accAngle, gyroRate, dt) :
        self.KFangle = self.KFangle + dt * (gyroRate - self.bias)

        self.P_00 = self.P_00 + ( - dt * (self.P_10 + self.P_01) + self.Q_angle * dt )
        self.P_01 = self.P_01 + ( - dt * self.P_11 )
        self.P_10 = self.P_10 + ( - dt * self.P_11 )
        self.P_11 = self.P_11 + ( + self.Q_gyro * dt )

        y = accAngle - self.KFangle
        S = self.P_00 + self.R_angle
        K_0 = self.P_00 / S
        K_1 = self.P_10 / S

        self.KFangle = self.KFangle + ( K_0 * y )
        self.bias = self.bias + ( K_1 * y )

        self.P_00 = self.P_00 - ( K_0 * self.P_00 )
        self.P_01 = self.P_01 - ( K_0 * self.P_01 )
        self.P_10 = self.P_10 - ( K_1 * self.P_00 )
        self.P_11 = self.P_11 - ( K_1 * self.P_01 )
        
        return self.KFangle
        
    def get(self) :
        return self.KFangle
    