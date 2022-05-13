import numpy as np
import measurement
import skrf
complex2db = lambda s: np.log10(np.mag(s))*20

class Record:
    def __init__(self, freq, s11, plot_line, s11_complex):
        self.data = [freq, s11, s11_complex]
        self.line = plot_line
    
    def calculate_q(self):
        freq = self.data[0]
        s11 = self.data[1]
        s11_complex = self.data[2]
        min_index = np.argmin(s11)
        freq_0 = freq[min_index]
        
        # build network object and convert to z
        start_f = freq[0]
        stop_f = freq[-1]
        n_f = len(freq)
        net = skrf.Network(s = s11_complex, frequency = skrf.Frequency(start_f, stop_f, n_f))
        z11 = net.z[:, 0, 0]
        z11_0 = z11[min_index]
        
        # Calculate r, x, deriv_r, deriv_x, omega_0
        r = np.real(z11_0)
        r_prev = np.real(z11[min_index-1])
        r_next = np.real(z11[min_index+1])
        x = np.imag(z11_0)
        x_prev = np.imag(z11[min_index-1])
        x_next = np.imag(z11[min_index+1])
        delta_freq = freq[1] - freq[0]
        deriv_r = (r_next - r_prev) / delta_freq / 2
        deriv_x = (x_next - x_prev) / delta_freq / 2
        omega_0 = 2*np.pi*freq_0
        
        # Calculate q
        q = omega_0/2/r*np.sqrt(deriv_r**2 + (deriv_x+x/omega_0)**2)
        return q
    
    def calculate_bandwidth(self):
        freq = self.data[0]
        s11 = self.data[1]
        min_index = np.argmin(s11)
        if s11[min_index] > -10:
            print('Warning: S11 is not matched below -10dB.')
            return 0
        else:
            bw_low_list = []
            bw_high_list = []
            for i in range(0, len(freq)):
                if s11[i] > -10.0:
                    if freq[i] < freq[min_index]:
                        bw_low = freq[min_index] - freq[i]
                        bw_low_list.append(bw_low)
                    elif freq[i] > freq[min_index]:
                        bw_high = freq[i] - freq[min_index]
                        bw_high_list.append(bw_high)
            
            if  bw_low_list == []:
                bw_low = 0
            else:
                bw_low = max(bw_low_list)
            
            if bw_high_list == []:
                bw_high = 0
            else:
                bw_high = min(bw_high_list)
            
            bw = bw_low + bw_high
            
            return bw/1e6
    
class RecordList:
    def __init__(self):
        self.record_list = []
    
    def append_record(self, rec):
        self.record_list.append(rec)
    
    def append_data(self, freq, s11, plot_line, s11_complex):
        self.record_list.append(Record(freq, s11, plot_line, s11_complex))
    
    def remove_all_record(self):
        self.record_list.clear()
        
    def remove_record(self, index):
        try:
            self.record_list.pop(index)
        except:
            print("Error: Cannot Remove Record with Index{}".format(index))
            pass

    def get_data_list(self):
        return self.record_list

    def get_plot_lines(self):
        return_list = []
        for record in self.record_list:
            return_list.append(record.line)
        return return_list
