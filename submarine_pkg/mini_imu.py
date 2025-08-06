import json
import math
import os
from smbus2 import SMBus
import time

def signed_u2_decode(data):
    """
    Takes Little-Endian values and converts them to signed int
    """
    octets = len(data) #liczba bajtow
    val = 0

    #zlozenie bajtow w liczbe - zaczynajac od najstarszych
    for byte in reversed(data): #reversed odwraca bajty bo dane sa w little endian
        val = (val << 8) | byte

    #sprawdzenie bit znaku jest ustawiony (czy liczba jest ujemna)
    if(val >= (1 << (octets*8 - 1))):
        val -= 1<<(octets*8) #jesli tak konwersja z unsigned na signed

    return val
    
def read_vector(bus, address, register):
    """
    Reads 3 axis data from sensor connected by i2c.
    Each axis value is represented by 2 bytes.
    """

    #odczyt 6 bajtow danych z czujnika, po 2 bajty na os
    data = bus.read_i2c_block_data(address, register, 6)

    #konwersja odczytanych danych z unsigned U2 na signed int
    x = signed_u2_decode(data[0:2]) #bajty X_L, X_H
    y = signed_u2_decode(data[2:4]) # Y_L, Y_H
    z = signed_u2_decode(data[4:6]) # Z_L, Z_H

    return x, y, z

class MiniIMU:

    def __init__(self, bus_id):

        #adresy czujnikow
        self.ACC_GYRO_ADDR = 0x6b #adres akcelerometr + gyro LSM6DS33
        self.MAG_ADDR = 0x1e #adres magnetometru LIS3MDL

        #rejestry konfig.
        self.CTRL1_XL = 0x10 #akcelerometr
        self.CTRL2_G = 0x11 #zyroskop
        self.CTRL9_XL = 0x18 #wlaczanie osi XYZ dla accel.
        self.CTRL10_C = 0x19 #wlaczanie osi XYZ dla gyro.
        self.CTRL_REG1_M = 0x20 #konfiguracja pracy magnetometru
        
        #rejestry danych wyj.
        self.OUTX_L_XL = 0x28 #dane z akcelerometru
        self.OUTX_L_G = 0x22 #dane z zyroskopu
        self.OUTX_L_M = 0x28 #dane z magnetometru

        self.INT1_CTRL = 0x0d #cos z przerwaniem?
        
        #domyslne biasy
        self.accel_bias = [0,0,0]
        self.gyro_bias = [0,0,0]
        
        # Nowe parametry filtrów
        self.pitch_filtered = 0.0
        self.roll_filtered = 0.0
        self.yaw_filtered = 0.0
        self.accel_pitch_filtered = 0.0
        self.accel_roll_filtered = 0.0
        self.gyro_drift_compensation = 0.0
        
        # Gyro integration values
        self.gyro_pitch = 0.0
        self.gyro_roll = 0.0
        self.gyro_yaw = 0.0
        
        # Timing for integration
        self._last_time = None
        
        # Historia odczytów do detekcji ruchu
        self.accel_history = []
        self.max_history_size = 10

        #sciezka do pliku z wart. z kalibracji - bezwzględna ścieżka względem tego pliku
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.calib_file_path = os.path.join(script_dir, 'calibration.json')

        self.bus = SMBus(bus_id)#inicjaliacja obiektu magistrali I2C

        #procedura inicjalizacji akceletrometru opisana w:
        # https://www.pololu.com/file/0J1088/LSM6DS33-AN4682.pdf
            
        #inicjalizacja akcel.
        #zalaczenie osi xyz accelerometru
        self.bus.write_byte_data(self.ACC_GYRO_ADDR, self.CTRL9_XL, 0x38)

        # Zredukowana częstotliwość próbkowania dla lepszej stabilności
        # 0100 0000 = 104 Hz 
        self.bus.write_byte_data(self.ACC_GYRO_ADDR, self.CTRL1_XL, 0b01000000)

        #ustawienie przerwania gdy dane sa gotowe na INT1 (opcjonalne?)
        self.bus.write_byte_data(self.ACC_GYRO_ADDR, self.INT1_CTRL, 0x02)

        #inicjalizacja zyroskopu
        #zalaczenie osi xyz zyroskopu
        self.bus.write_byte_data(self.ACC_GYRO_ADDR, self.CTRL10_C, 0x38)

        # Zredukowana częstotliwość dla żyroskopu: 104 Hz
        self.bus.write_byte_data(self.ACC_GYRO_ADDR, self.CTRL2_G, 0x40)

        #zalaczenie przerwania na INT1 gdy dane sa gotowe
        self.bus.write_byte_data(self.ACC_GYRO_ADDR, self.INT1_CTRL, 0x02)

        #inicjalizacja magnetometru
        #wyslanie ustawien na rejestr konf. mag
        #0x70 = 0111 0000-> kompensacja temp, ultra-high perf., ODR = 10Hz
        self.bus.write_byte_data(self.MAG_ADDR, self.CTRL_REG1_M, 0x70)


    def read_raw_data(self):
        """
        Reads raw data from accelerometer, gyro and magnetometer sensors.
        Returns 3 Vectors for X,Y,Z axis as a tuple (x, y, z)
        """
        acc = read_vector(self.bus, self.ACC_GYRO_ADDR, self.OUTX_L_XL)
        gyro = read_vector(self.bus, self.ACC_GYRO_ADDR, self.OUTX_L_G) 
        mag = read_vector(self.bus, self.MAG_ADDR, self.OUTX_L_M) 

        return [acc, gyro, mag]
    

    def read_data(self, scaled = True):
        """
        Reads data from sensor but corrected by biases from calibration.
        Returns 3 Vectors for X,Y,Z axis as a tuple (x, y, z) 
        If scaled param = true then returned values are scaled to physical units
        """
        acc_raw, gyro_raw, mag_raw = self.read_raw_data()
        
        accel = [acc_raw[i] - self.accel_bias[i] for i in range(3)]
        gyro = [gyro_raw[i] - self.gyro_bias[i] for i in range(3)]

        if scaled:
            #if the accelerometer scale-selection is set to +-2g then
            # raw value: +32768 eq to +2g .. -> +16384 = +1g
            accel = [axis / 16384.0 for axis in accel]

            #gyro scale selection = +-245deg/s -> 1deg/s = 131
            gyro = [axis / 131.0 for axis in gyro]

        return [accel, gyro, mag_raw]


    def calibration(self, sample_count):
        """
        takes x samples,
        cuts off the extremal values, then calculate avg of them to get bias for each axis for accel. and gyro
        Saves biases to [calibration.json] file.
        
        """
        accel_samples = []
        gyro_samples = []

        print("Calibration started - keep sensor steady")

        for i in range(sample_count):
            sample = self.read_raw_data()
            accel_samples.append(sample[0])
            gyro_samples.append(sample[1])
            
            
            print(f"Sample: {i}/{sample_count}")
            
            time.sleep(0.01)

        #usun wartosci ekstremalne -10% z kazdej strony
        def remove_outliers(samples):
            axis_samples = [[], [], []]
            for sample in samples:
                for axis in range(3):
                    axis_samples[axis].append(sample[axis])
            
            filtered_samples = []
            for axis in range(3):
                sorted_axis = sorted(axis_samples[axis])
                #usun 10% najwyzszych i najnizszych odczytow
                trim_count = len(sorted_axis) // 10
                if trim_count > 0:
                    trimmed = sorted_axis[trim_count:-trim_count]
                    axis_samples[axis] = trimmed
            
            #obliczanie srednich
            return [sum(axis_samples[i]) / len(axis_samples[i]) for i in range(3)]

        calc_accel_bias = remove_outliers(accel_samples)
        calc_gyro_bias = remove_outliers(gyro_samples)

        #orientacja z skierowane w dol
        calc_accel_bias[2] = calc_accel_bias[2] - 16384  #-1g od biasu Z

        print("Calibration finished")
        print(f"Accel bias: {calc_accel_bias}")
        print(f"Gyro bias: {calc_gyro_bias}")
        print(f"Saving calibration to: {self.calib_file_path}")
        
        with open(self.calib_file_path, 'w') as file:
            json.dump({
                'accel_bias': calc_accel_bias,
                'gyro_bias': calc_gyro_bias
            }, file, indent=2)


    def load_calibration(self):
        """
        Loads calibrated biases and sets them to object attr.
        """
        print(f"Looking for calibration file at: {self.calib_file_path}")
        print(f"Current working directory: {os.getcwd()}")
        
        try:
            with open(self.calib_file_path) as file:
                data = json.load(file)
                self.accel_bias = data.get('accel_bias', [0,0,0])
                self.gyro_bias = data.get('gyro_bias', [0,0,0])
                print(f'loaded calibration - accel: {self.accel_bias}, gyro: {self.gyro_bias}')
        except FileNotFoundError:
            print(f"Calibration file cannot be found at: {self.calib_file_path}")
            print("Make calibration before")


    def normalize_angle(self, angle):
        """
        Normalizes angles to +-180
        """
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def is_device_stationary(self, accel_data, threshold=0.1):
        """
        Checks if device is stationary based on measurements history
        """
        #dodaj nowy odczyt do historii
        self.accel_history.append(accel_data)
        
        #ogranicz rozmiar historii
        if len(self.accel_history) > self.max_history_size:
            self.accel_history.pop(0)
        
        # jesli nie mam wystarczającej historii, to zakladamy ruch
        if len(self.accel_history) < self.max_history_size:
            return False
        
        # oblicz wariancję dla każdej osi
        variances = []
        for axis in range(3):
            values = [reading[axis] for reading in self.accel_history]
            mean = sum(values) / len(values)
            variance = sum((x - mean) ** 2 for x in values) / len(values)
            variances.append(variance)
        
        #sensor jest w spoczynku jesli wariancje są male, zwraca true
        return all(var < threshold for var in variances)

    def _calculate_dt(self):
        """Calculate time delta for integration"""
        now = time.time()
        if not hasattr(self, '_last_time') or self._last_time is None:
            self._last_time = now
            return 0.02  # default 20ms
        
        dt = now - self._last_time
        dt = max(0.005, min(dt, 0.1))  # min 5ms, max 100ms
        self._last_time = now
        return dt

    def _calculate_accel_angles(self, accel):
        """Calculate pitch and roll from accelerometer data"""
        pitch = math.atan2(accel[1], accel[2]) * 180.0 / math.pi
        roll = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180.0 / math.pi
        return pitch, roll

    def _apply_low_pass_filter(self, new_value, filtered_value, alpha=0.2):
        """Apply low-pass filter to reduce noise"""
        return alpha * new_value + (1 - alpha) * filtered_value

    def _get_adaptive_alpha(self, accel, is_stationary, base_alpha=0.95):
        """Calculate adaptive alpha based on motion and acceleration magnitude"""
        accel_magnitude = math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
        
        if is_stationary and abs(accel_magnitude - 1.0) < 0.2:
            # device stationary and gravity ~1g - trust accelerometer
            return 0.85  # more accelerometer
        elif abs(accel_magnitude - 1.0) > 0.5:
            # high accelerations - trust gyroscope
            return 0.99
        else:
            # normal motion
            return base_alpha

    def _complementary_filter(self, gyro_angle, accel_angle, alpha):
        """Apply complementary filter with wrap-around handling"""
        # normalize angles to -180 to +180 range
        gyro_angle = self.normalize_angle(gyro_angle)
        accel_angle = self.normalize_angle(accel_angle)
        
        # handle wrap-around (transition through ±180°)
        angle_diff = accel_angle - gyro_angle
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # improved complementary filter - eliminates long-term drift
        gyro_angle += (1 - alpha) * angle_diff
        return self.normalize_angle(gyro_angle)

    def _initialize_angles(self):
        """Initialize all angle-related attributes if not present"""
        if not hasattr(self, 'angles_initialized') or not self.angles_initialized:
            # get initial accelerometer reading for proper initialization
            accel, gyro, mag = self.read_data(scaled=True)
            initial_pitch, initial_roll = self._calculate_accel_angles(accel)
            
            # initialize all gyro angles
            self.gyro_pitch = initial_pitch
            self.gyro_roll = initial_roll
            self.gyro_yaw = 0.0  # yaw starts at 0 (no reference from accelerometer)
            
            # initialize filtered values  
            self.accel_pitch_filtered = initial_pitch
            self.accel_roll_filtered = initial_roll
            self.pitch_filtered = initial_pitch
            self.roll_filtered = initial_roll
            self.yaw_filtered = 0.0
            
            # timing
            self._last_time = None
            
            self.angles_initialized = True
            print(f"Kąty zainicjalizowane: Pitch={initial_pitch:.1f}°, Roll={initial_roll:.1f}°")

    def get_all_angles(self, dt=None, alpha=0.95):
        """
        GŁÓWNA FUNKCJA - pobiera wszystkie trzy kąty (pitch, roll, yaw) w jednym wywołaniu
        Ta funkcja powinna być używana zamiast osobnych get_pitch/get_roll/get_yaw
        """
        # inicjalizacja przy pierwszym wywołaniu
        self._initialize_angles()
        
        # odczytaj dane z sensorów
        accel, gyro, mag = self.read_data(scaled=True)
        
        # oblicz dt (tylko raz dla wszystkich kątów!)
        if dt is None:
            dt = self._calculate_dt()
        
        # oblicz kąty z akcelerometru
        accel_pitch, accel_roll = self._calculate_accel_angles(accel)
        
        # sprawdź czy sensor jest w spoczynku
        is_stationary = self.is_device_stationary(accel)
        
        # oblicz adaptacyjne alpha
        effective_alpha = self._get_adaptive_alpha(accel, is_stationary, alpha)
        
        # === PITCH (obrót wokół osi X) ===
        # filtruj odczyt akcelerometru
        self.accel_pitch_filtered = self._apply_low_pass_filter(
            accel_pitch, self.accel_pitch_filtered, alpha=0.2)
        
        # integracja żyroskopu
        self.gyro_pitch += gyro[0] * dt
        
        # filtr komplementarny
        self.gyro_pitch = self._complementary_filter(
            self.gyro_pitch, self.accel_pitch_filtered, effective_alpha)
        
        # wygładzanie wyjścia
        self.pitch_filtered = self._apply_low_pass_filter(
            self.gyro_pitch, self.pitch_filtered, 0.9)
        
        # === ROLL (obrót wokół osi Y) ===
        # filtruj odczyt akcelerometru
        self.accel_roll_filtered = self._apply_low_pass_filter(
            accel_roll, self.accel_roll_filtered, alpha=0.2)
        
        # integracja żyroskopu (roll to oś Y)
        self.gyro_roll += gyro[1] * dt
        
        # filtr komplementarny
        self.gyro_roll = self._complementary_filter(
            self.gyro_roll, self.accel_roll_filtered, effective_alpha)
        
        # wygładzanie wyjścia
        self.roll_filtered = self._apply_low_pass_filter(
            self.gyro_roll, self.roll_filtered, 0.9)
        
        # === YAW (obrót wokół osi Z) ===
        # tylko integracja żyroskopu (brak referencji z akcelerometru)
        self.gyro_yaw += gyro[2] * dt
        self.gyro_yaw = self.normalize_angle(self.gyro_yaw)
        
        # wygładzanie wyjścia
        self.yaw_filtered = self._apply_low_pass_filter(
            self.gyro_yaw, self.yaw_filtered, 0.9)
        
        # debug info
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 50 == 0:  # co 50 próbek
            accel_magnitude = math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
            print(f'dt:{dt:.3f}s | stationary:{is_stationary} | accel_mag:{accel_magnitude:.2f}g | alpha:{effective_alpha:.2f}')
            print(f'Raw accel: X:{accel[0]:.2f}g Y:{accel[1]:.2f}g Z:{accel[2]:.2f}g')
            print(f'Raw gyro: X:{gyro[0]:.1f}°/s Y:{gyro[1]:.1f}°/s Z:{gyro[2]:.1f}°/s')
        
        return self.pitch_filtered, self.roll_filtered, self.yaw_filtered

    def get_pitch(self, dt=None, alpha=0.95):
        """PRZESTARZAŁE - użyj get_all_angles() dla lepszej wydajności"""
        pitch, _, _ = self.get_all_angles(dt, alpha)
        return pitch

    def get_roll(self, dt=None, alpha=0.95):
        """PRZESTARZAŁE - użyj get_all_angles() dla lepszej wydajności"""
        _, roll, _ = self.get_all_angles(dt, alpha)
        return roll

    def get_yaw(self, dt=None):
        """PRZESTARZAŁE - użyj get_all_angles() dla lepszej wydajności"""
        _, _, yaw = self.get_all_angles(dt)
        return yaw   


if __name__ == '__main__':
    
    imu = MiniIMU(1)

    print("Performing new calibration...")
    
    imu.calibration(4000)
    imu.load_calibration()
    

    print("Reading IMU data:")
    print("First few readings may be unstable during filter initialization...")
    
    print("Odczyt kątów Eulera:")
    for i in range(200):
        pitch, roll, yaw = imu.get_all_angles()  # jedna funkcja!
        print(f'Pitch:{pitch:6.1f}° Roll:{roll:6.1f}° Yaw:{yaw:6.1f}°')
        time.sleep(0.05)