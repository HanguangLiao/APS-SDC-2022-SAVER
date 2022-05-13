import numpy
import skrf
import pocketvna

import matplotlib
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, 
NavigationToolbar2Tk)

import time

class Measurement:
    def __init__(self):
        # Frequency profile
        self.start_freq_ghz = 2.0
        self.stop_freq_ghz = 3.0
        self.n_freq = 1001
        self.frequency = numpy.linspace(self.start_freq_ghz, self.stop_freq_ghz, self.n_freq)
        self.frequency_list = list(self.frequency*1E9)
        self._skrf_freq = skrf.Frequency.from_f(self.frequency, unit='ghz')
        
        # Calibration Profile
        self.cal_root_folder = 'Calibration'
        self.cal_ideal_short = []
        self.cal_ideal_open = []
        self.cal_ideal_load = []
        self.cal_ideal_solt = [self.cal_ideal_short, self.cal_ideal_open, self.cal_ideal_load, []]       
        self.cal_meas_solt = []
        # Measurement Profile
        self.z0 = 50.0
        self.n_avg = 2
        pass
    
    def update_frequency_profile(self, f_start_ghz, f_stop_ghz, n_freq):
        self.start_freq_ghz = f_start_ghz
        self.stop_freq_ghz = f_stop_ghz
        self.n_freq = n_freq
        self.frequency = numpy.linspace(self.start_freq_ghz, self.stop_freq_ghz, n_freq)
        self.frequency_list = list(self.frequency*1E9)
        self._skrf_freq = skrf.Frequency.from_f(self.frequency, unit='ghz')
    
    def update_cal_folder(self, folderpath):
        self.cal_root_folder = folderpath
        pass
    
    def update_solt_ideal_network(self):
        # Load Ideal Data of Calibration Standards and Interpolate into the Required Frequency Profile
        try:
            # Ideal Short, Open, and Load
            port_names = ("port1", "port2")
            one_port_standard_names = ("short", "open", "load")
            self._cal_ideal_one_ports = []
            for port_name in port_names:
                new_port = []
                for standard_name in one_port_standard_names:
                    new_port.append(skrf.Network(self.cal_root_folder + '/IdealStandards/' + port_name + '_' + standard_name + '.s1p').interpolate_from_f(self._skrf_freq))
                self._cal_ideal_one_ports.append(new_port)
            # Ideal Thru
            self._cal_ideal_one_ports.append(skrf.Network(self.cal_root_folder + '/IdealStandards/' + 'thru' + '.s2p').interpolate_from_f(self._skrf_freq))
            
            # Ideal Calibration SOLT Networks
            self.cal_ideal_short = skrf.two_port_reflect(self._cal_ideal_one_ports[0][0],self._cal_ideal_one_ports[1][0])
            self.cal_ideal_open = skrf.two_port_reflect(self._cal_ideal_one_ports[0][1],self._cal_ideal_one_ports[1][1])
            self.cal_ideal_load = skrf.two_port_reflect(self._cal_ideal_one_ports[0][2],self._cal_ideal_one_ports[1][2])
            self.cal_ideal_solt = [self.cal_ideal_short, self.cal_ideal_open, self.cal_ideal_load, self._cal_ideal_one_ports[2]]       

        except:
            print('Error: Cannot Load Ideal Calibration Files')
            return False
        
        # Create the list structure for storing measured standards
        self._cal_meas_one_ports = []
        for port_name in port_names:
            new_port = []
            for standard_name in one_port_standard_names:
                new_port.append(port_name + '_' + standard_name)
            self._cal_meas_one_ports.append(new_port)
        self._cal_meas_one_ports.append('thru')        
        return True
    
    def update_solt_measurement_network(self, port='port1', standard='short'):
        port_names = ("port1", "port2", "thru")
        one_port_standard_names = ("short", "open", "load")
        try:
            # Get the index of port and standard in stored one-port list structure
            for port_idx in range(0, len(port_names)):
                if(port == port_names[port_idx]):
                    break
                else:
                    continue
            if port_idx <= 1:
                # update short, or open, or load of port 1 or port 2
                for standard_idx in range(0, len(one_port_standard_names)):
                    if(standard == one_port_standard_names[standard_idx]):
                        break
                    else:
                        continue
                self._cal_meas_one_ports[port_idx][standard_idx] = skrf.Network(self.cal_root_folder + '/MeasurementStandards/' + port + '_' + standard + '.s1p').interpolate_from_f(self._skrf_freq)
            else:
                # update thru
                self._cal_meas_one_ports[port_idx] = skrf.Network(self.cal_root_folder + '/MeasurementStandards/' + 'thru' + '.s2p').interpolate_from_f(self._skrf_freq)
        except:
            print('Error: Cannot Load Measured ' + port + '_' + standard)
            return False
        return True
    
    def write_1port_file_from_list(self, s1p_list, filename):
        net = skrf.Network(s=s1p_list.reshape((self.n_freq, 1,1)), frequency=self._skrf_freq, z0 = self.z0)
        net.write_touchstone(filename)
    
    def write_2port_file_from_lists(self, s11, s12, s21, s22, filename):
        s_mat = numpy.zeros((self.n_freq, 2,2), dtype=numpy.complex128)
        s_mat[:, 0, 0] = numpy.array(s11, dtype=numpy.complex128)
        s_mat[:, 1, 0] = numpy.array(s21, dtype=numpy.complex128)
        s_mat[:, 0, 1] = numpy.array(s12, dtype=numpy.complex128)
        s_mat[:, 1, 1] = numpy.array(s22, dtype=numpy.complex128)
        net = skrf.Network(s=s_mat, frequency=self._skrf_freq, z0=self.z0)
        net.write_touchstone(filename)
    
    def create_solt_measurement_network(self, driver, port='port1', standard='short'):
        try:
            time_start = time.time()
            if port == 'port1':
                s11, s21, s12, s22 = driver.scan(self.frequency_list, self.n_avg, pocketvna.NetworkParams.S11)
                self.write_1port_file_from_list(s11, self.cal_root_folder + '/MeasurementStandards/' + port + '_' + standard)
            elif port == 'port2':
                s11, s21, s12, s22 = driver.scan(self.frequency_list, self.n_avg, pocketvna.NetworkParams.S22)
                self.write_1port_file_from_list(s22, self.cal_root_folder + '/MeasurementStandards/' + port + '_' + standard)
            else:
                s11, s21, s12, s22 = driver.scan(self.frequency_list, self.n_avg, pocketvna.NetworkParams.AllSupported)
                self.write_2port_file_from_lists(s11, s12, s21, s22, self.cal_root_folder + '/MeasurementStandards/' + 'thru')
            time_stop = time.time()
            print("Measurement Takes {:.1f} seconds".format(time_stop - time_start))
        except:
            print("Error: Cannot Create Measured " + port + '_' + standard)
            return False
        return True

    def update_solt_measurement_network_list(self):
        try:
            # Measured Calibration SOLT Networks
            self._cal_meas_short = skrf.two_port_reflect(self._cal_meas_one_ports[0][0],self._cal_meas_one_ports[1][0])
            self._cal_meas_open = skrf.two_port_reflect(self._cal_meas_one_ports[0][1],self._cal_meas_one_ports[1][1])
            self._cal_meas_load = skrf.two_port_reflect(self._cal_meas_one_ports[0][2],self._cal_meas_one_ports[1][2])
            self.cal_meas_solt = [self._cal_meas_short, self._cal_meas_open, self._cal_meas_load, self._cal_meas_one_ports[2]]       
            self._cal = skrf.calibration.SOLT(ideals=self.cal_ideal_solt, measured=self.cal_meas_solt)
            self._cal.run()
        except:
            print("Error: Calibration is invalid")
            return False
        return True
    
    def build_network_from_lists(self, s11, s12, s21, s22):
        s_mat = numpy.zeros((self.n_freq, 2,2), dtype=numpy.complex128)
        s_mat[:, 0, 0] = numpy.array(s11, dtype=numpy.complex128)
        s_mat[:, 1, 0] = numpy.array(s21, dtype=numpy.complex128)
        s_mat[:, 0, 1] = numpy.array(s12, dtype=numpy.complex128)
        s_mat[:, 1, 1] = numpy.array(s22, dtype=numpy.complex128)
        net = skrf.Network(s=s_mat, frequency=self._skrf_freq, z0=self.z0)
        return net
    
    def measure_and_cal_network(self, driver):
        time_start = time.time()
        # Measure S Vectors Using VNA Driver
        s11, s21, s12, s22 = driver.scan(self.frequency_list, self.n_avg, pocketvna.NetworkParams.AllSupported)
        
        time_stop = time.time()
        print("Measurement Takes {:.1f} seconds".format(time_stop - time_start))
        
        # Build SKRF Network Object
        net_raw = self.build_network_from_lists(s11, s12, s21, s22)
        # Run Calibration
        net_calibrated = self._cal.apply_cal(net_raw)
        return net_calibrated

class LiveMeasurement:
    def __init__(self, measurement):
        self.is_measurement_complete = False
        self.is_stop_request = False
        self._n_freq_per_segment = 4
        self._n_segment = measurement.n_freq // self._n_freq_per_segment + 1
        self._i_segment = 0
        pass
    
    def setup_new_measurement(self, driver, measurement):
        self.is_measurement_complete = False
        self.is_stop_request = False
        if measurement.n_freq % self._n_freq_per_segment == 0:
            self._n_segment = measurement.n_freq // self._n_freq_per_segment
        else:
            self._n_segment = measurement.n_freq // self._n_freq_per_segment + 1
        self._i_segment = 0
        pass
    
    def stop_measurement(self, driver, measurement):
        self.is_stop_request = True
        pass
    
    def scan_segment_s11(self, driver, measurement):
        time_start = time.time()
        # Each Sample Takes 0.023 Seconds
        i_seg = self._i_segment  # Idex of Current Measurement Segment, Ranging from 0 to n_seg-1
        n_seg = self._n_segment  # Total Number of Segments
        n_freq_in_seg = self._n_freq_per_segment
        # Generate SKRF Frequency Object
        if i_seg < n_seg-1:
            freq_list = measurement.frequency_list[i_seg*n_freq_in_seg:(i_seg+1)*n_freq_in_seg]
        else:
            freq_list = measurement.frequency_list[i_seg*n_freq_in_seg::]
            
        n_freq_in_seg = len(freq_list)
        
        skrf_freq = skrf.Frequency(freq_list[0], freq_list[-1], len(freq_list), unit='hz')
        
        # Re-interpolate Calibration SOLTs
        cal_ideal_solt = map(lambda solt: solt.interpolate_from_f(skrf_freq), measurement.cal_ideal_solt)
        cal_meas_solt = map(lambda solt: solt.interpolate_from_f(skrf_freq), measurement.cal_meas_solt)
        cal = skrf.calibration.SOLT(ideals=list(cal_ideal_solt), measured=list(cal_meas_solt))
        cal.run()
        
        # Measure Samples
        s11, s21, s12, s22 = driver.scan(freq_list, measurement.n_avg, pocketvna.NetworkParams.AllSupported)
        s_mat = numpy.zeros((n_freq_in_seg, 2,2), dtype=numpy.complex128)
        s_mat[:, 0, 0] = numpy.array(s11, dtype=numpy.complex128)
        s_mat[:, 1, 0] = numpy.array(s21, dtype=numpy.complex128)
        s_mat[:, 0, 1] = numpy.array(s12, dtype=numpy.complex128)
        s_mat[:, 1, 1] = numpy.array(s22, dtype=numpy.complex128)
        net = skrf.Network(s=s_mat, frequency=skrf_freq, z0=measurement.z0)
        
        # Calibrate Samples
        seg_network = cal.apply_cal(net)
        
        # When Sampling is finished, update the state variable
        self._i_segment = self._i_segment + 1
        if self._i_segment >= self._n_segment:
            self.is_measurement_complete = True
            
        time_stop = time.time()
        
        ## S11 From Network
        seg_s11 = seg_network.s[:, 0, 0]
        db_seg_s11 = list(numpy.log10(numpy.abs(seg_s11)) * 20)
        
        print("Segment {} Takes {:.1f} Seconds.".format(i_seg, time_stop - time_start))
        
        return db_seg_s11, freq_list, seg_s11
