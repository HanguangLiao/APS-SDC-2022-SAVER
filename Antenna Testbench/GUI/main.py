import pathlib
import tkinter as tk
import tkinter.ttk as ttk
from tkinter import simpledialog
import pygubu


import pocketvna
import measurement as meas
import skrf

import numpy
import matplotlib
import matplotlib.style as mplstyle
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)

import time

import records

PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH / "SaverMainApp.ui"


class SavermainappApp:
    def __init__(self, master=None):
        ## GUI Init
        time_start = time.time()
        
        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)
        builder.add_from_file(PROJECT_UI)
        self.mainwindow = builder.get_object('main_window', master)
        
        self.cal_folder_path = None
        self.sweep_start_freq_ghz = None
        self.sweep_stop_freq_ghz = None
        self.sweep_step_freq_ghz = None
        self.help_text = None
        builder.import_variables(self, ['cal_folder_path', 'sweep_start_freq_ghz', 'sweep_stop_freq_ghz', 'sweep_n_freq', 'help_text'])
        
        builder.connect_callbacks(self)
        
        # Set ttk Classical Style
        ttk.Style().theme_use('clam')
        
        # Initiate Live Canvas
        mplstyle.use('fast')
        matplotlib.use('TkAgg')
        self.fig_live = plt.figure(1)
        self.canvas_frame = self.builder.get_object('frame_live_canvas')
        self.live_canvas = FigureCanvasTkAgg(self.fig_live, master=self.canvas_frame)
        self.live_canvas.get_tk_widget().pack(side=tk.TOP, anchor=tk.N, fill=tk.BOTH, expand=1)

        time_stop = time.time()
        print("Window Loading Takes {:.1f} seconds".format(time_stop - time_start))
        
    def run(self):
        self.mainwindow.mainloop()

    def callback_list_device(self, event=None):
        dev_lst, dev_size = driver.enumerate()
        dev_stat = self.builder.get_object('label_device_status_text')
        dev_combolist = self.builder.get_object('combobox_device_list')
        dev_combolist['values'] = []
        if dev_size > 0:
            for i in range(0, dev_size):
                dev_combolist['values'] = list(dev_combolist['values']) + ["No.{} - ".format(i) + str(dev_lst[i])]
            dev_combolist.current(0)
            dev_stat['text'] = "Device List Updated"
        else:
            print("Error: Cannot find pocketVNA device.")
            dev_stat['text'] = "Cannot find pocketVNA device."
        pass

    def callback_connect_device(self, event=None):
        dev_stat = self.builder.get_object('label_device_status_text')
        dev_combolist = self.builder.get_object('combobox_device_list')
        #ignore repetitive connection
        if driver.valid():
            dev_stat["text"] = "Valid connection exists"
        else:
            dev_stat["text"] = "Preparing to Connect Device No.{}".format(dev_combolist.current())
            if driver.safe_connect_to(dev_combolist.current()):
                dev_stat["text"] = "Connection is successful"
            else:
                dev_stat["text"] = "Connection failed"
        ## clear previous list info
        dev_property = self.builder.get_object('treeview_property')
        for item in dev_property.get_children():
                dev_property.delete(item)        
        ## list device info when it is available
        if driver.valid():
            ## list device info column
            dev_property['columns'] = ('Name', 'Value')
            dev_property.column('#0', width=100)
            dev_property.column('Name', width=200)
            dev_property.column('Value', width=200)
            dev_property.heading('#0', text='ID')
            dev_property.heading('Name', text='Name')
            dev_property.heading('Value', text='Value')
            ## list device info Meas Support
            nps = ''
            if driver.has_s11(): nps = nps + "S11 "
            if driver.has_s21(): nps = nps + "S21 "
            if driver.has_s12(): nps = nps + "S12 "
            if driver.has_s22(): nps = nps + "S22 "
            dev_property.insert('', tk.END, text=str(0), values=('Measurement Support', nps))
            ## list device info Firmware Version
            dev_property.insert('', tk.END, text=str(1), values=('Firmware Version', driver.version()))
            ## list device characteristic impedance
            dev_property.insert('', tk.END, text=str(2), values=('Z0', "{} Ohm".format(driver.Z0())))
            ## list device frequency range
            f_start, f_end = driver.reasonable_frequency_range()
            dev_property.insert('', tk.END, text=str(3), values=('Frequency Range', "{} MHz - {} GHz".format(f_start/1E6, f_end/1E9)))
        pass

    def callback_disconnect_device(self, event=None):
        ## Disconnect Device
        driver.close()
        dev_stat = self.builder.get_object('label_device_status_text')
        dev_stat["text"] = "Disconnected"
        
        ## clear device list
        dev_combolist = self.builder.get_object('combobox_device_list')
        dev_combolist['values'] = []
        
        ## clear previous list info
        dev_property = self.builder.get_object('treeview_property')
        for item in dev_property.get_children():
                dev_property.delete(item) 
        pass
    
    def callback_calibration_open_folder(self, event=None):
        folderpath = tk.filedialog.askdirectory(mustexist=True)
        if len(folderpath) > 0:
            self.builder.tkvariables['cal_folder_path'].set(folderpath)
            print("Calibration folder selected to " + folderpath)
        else:
            txt_status = self.builder.get_object('label_measurement_calibration_status_text')
            txt_status['text'] = "Invalid Calibration Folder"
        pass
    
    def callback_calibration_choose_folder(self, event=None):
        folderpath = self.builder.tkvariables['cal_folder_path'].get()
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        if len(folderpath) > 0:
            txt_status['text'] = "Calibration Folder Set to\n" + folderpath + "\nLoading Ideal Calibration Files..."
            self.mainwindow.update()
            measurement.update_cal_folder(folderpath)
            if measurement.update_solt_ideal_network():
                txt_status['text'] = "Ideal Calibration Files Loaded"
            else:
                txt_status['text'] = "Error: Cannot Load Ideal Calibration Files"
        else:
            txt_status['text'] = "Invalid Calibration Folder"
        pass
    
    def callback_apply_sweep(self, event=None):
        txt_status = self.builder.get_object('label_measurement_sweep_text')
        try:
            start_freq_ghz = float(self.builder.tkvariables['sweep_start_freq_ghz'].get())
            stop_freq_ghz = float(self.builder.tkvariables['sweep_stop_freq_ghz'].get())
            n_freq = int(self.builder.tkvariables['sweep_n_freq'].get())
        except ValueError:
            txt_status['text'] = "Sweep Parameter Error: Input format are not matched"
            return
        if(stop_freq_ghz <= start_freq_ghz):
            txt_status['text'] = "Sweep Parameter Error: Stop Freq <= Start Freq"
            return
        if(n_freq <= 2):
            txt_status['text'] = "Sweep Parameter Error: Number of Points <= 2"
            return
        f_step_ghz = (stop_freq_ghz - start_freq_ghz)/(n_freq - 1)
        txt_status['text'] = "Sweep Set from {} GHz to {} GHz, step {} GHz".format(start_freq_ghz, stop_freq_ghz, f_step_ghz)
        measurement.update_frequency_profile(start_freq_ghz, stop_freq_ghz, n_freq)
        pass
    
    def callback_recall_sweep(self, event=None):
        self.builder.tkvariables['sweep_start_freq_ghz'].set(measurement.start_freq_ghz)
        self.builder.tkvariables['sweep_stop_freq_ghz'].set(measurement.stop_freq_ghz)
        self.builder.tkvariables['sweep_n_freq'].set(measurement.n_freq)
        pass
    
    def callback_clear_sweep(self, event=None):
        self.builder.tkvariables['sweep_start_freq_ghz'].set('')
        self.builder.tkvariables['sweep_stop_freq_ghz'].set('')
        self.builder.tkvariables['sweep_n_freq'].set('')
        pass

    def callback_apply_p1s(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        label_status = self.builder.get_object('cal_label_p1s')
        if measurement.update_solt_measurement_network(port='port1', standard='short'):
            txt_status['text'] = 'Port 1 Short Standard Applied'
            label_status['state'] = 'normal'
        else:
            txt_status['text'] = 'Error: Cannot Apply Port 1 Short Standard'
            label_status['state'] = 'disabled'
        pass
    def callback_apply_p1o(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        label_status = self.builder.get_object('cal_label_p1o')
        if measurement.update_solt_measurement_network(port='port1', standard='open'):
            txt_status['text'] = 'Port 1 Open Standard Applied'
            label_status['state'] = 'normal'
        else:
            txt_status['text'] = 'Error: Cannot Apply Port 1 Open Standard'
            label_status['state'] = 'disabled'
        pass
    def callback_apply_p1l(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        label_status = self.builder.get_object('cal_label_p1l')
        if measurement.update_solt_measurement_network(port='port1', standard='load'):
            txt_status['text'] = 'Port 1 Load Standard Applied'
            label_status['state'] = 'normal'
        else:
            txt_status['text'] = 'Error: Cannot Apply Port 1 Load Standard'
            label_status['state'] = 'disabled'
        pass
    
    def callback_apply_p2s(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        label_status = self.builder.get_object('cal_label_p2s')
        if measurement.update_solt_measurement_network(port='port2', standard='short'):
            txt_status['text'] = 'Port 2 Short Standard Applied'
            label_status['state'] = 'normal'
        else:
            txt_status['text'] = 'Error: Cannot Apply Port 2 Short Standard'
            label_status['state'] = 'disabled'
        pass       
    def callback_apply_p2o(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        label_status = self.builder.get_object('cal_label_p2o')
        if measurement.update_solt_measurement_network(port='port2', standard='open'):
            txt_status['text'] = 'Port 2 Open Standard Applied'
            label_status['state'] = 'normal'
        else:
            txt_status['text'] = 'Error: Cannot Apply Port 2 Open Standard'
            label_status['state'] = 'disabled'
        pass 
    def callback_apply_p2l(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        label_status = self.builder.get_object('cal_label_p2l')
        if measurement.update_solt_measurement_network(port='port2', standard='load'):
            txt_status['text'] = 'Port 2 Load Standard Applied'
            label_status['state'] = 'normal'
        else:
            txt_status['text'] = 'Error: Cannot Apply Port 2 Load Standard'
            label_status['state'] = 'disabled'
        pass
    
    def callback_apply_thru(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        label_status = self.builder.get_object('cal_label_thru')
        if measurement.update_solt_measurement_network(port='thru', standard='thru'):
            txt_status['text'] = 'Thru Standard Applied'
            label_status['state'] = 'normal'
        else:
            txt_status['text'] = 'Error: Cannot Apply Thru Standard'
            label_status['state'] = 'disabled'
        pass 
    
    def callback_create_thru(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        txt_status['text'] = 'Measuring Thru Standard...'
        self.mainwindow.update()
        if measurement.create_solt_measurement_network(driver, port='thru', standard='thru'):
            txt_status['text'] = 'Thru Standard Created'
        else:
            txt_status['text'] = 'Error: Cannot Create Thru Standard'
        pass
    
    def callback_create_p1s(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        txt_status['text'] = 'Measuring Port 1 Short Standard...'
        self.mainwindow.update()
        if measurement.create_solt_measurement_network(driver, port='port1', standard='short'):
            txt_status['text'] = 'Port 1 Short Standard Created'
        else:
            txt_status['text'] = 'Error: Cannot Create Port 1 Short Standard'
        pass
    def callback_create_p1o(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        txt_status['text'] = 'Measuring Port 1 Open Standard...'
        self.mainwindow.update()
        if measurement.create_solt_measurement_network(driver, port='port1', standard='open'):
            txt_status['text'] = 'Port 1 Open Standard Created'
        else:
            txt_status['text'] = 'Error: Cannot Create Port 1 Open Standard'
        pass
    def callback_create_p1l(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        txt_status['text'] = 'Measuring Port 1 Load Standard...'
        self.mainwindow.update()
        if measurement.create_solt_measurement_network(driver, port='port1', standard='load'):
            txt_status['text'] = 'Port 1 Load Standard Created'
        else:
            txt_status['text'] = 'Error: Cannot Create Port 1 Load Standard'
        pass
    
    def callback_create_p2s(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        txt_status['text'] = 'Measuring Port 2 Short Standard...'
        self.mainwindow.update()
        if measurement.create_solt_measurement_network(driver, port='port2', standard='short'):
            txt_status['text'] = 'Port 2 Short Standard Created'
        else:
            txt_status['text'] = 'Error: Cannot Create Port 2 Short Standard'
        pass
    def callback_create_p2o(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        txt_status['text'] = 'Measuring Port 2 Open Standard...'
        self.mainwindow.update()
        if measurement.create_solt_measurement_network(driver, port='port2', standard='open'):
            txt_status['text'] = 'Port 2 Open Standard Created'
        else:
            txt_status['text'] = 'Error: Cannot Create Port 2 Open Standard'
        pass
    def callback_create_p2l(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        txt_status['text'] = 'Measuring Port 2 Load Standard...'
        self.mainwindow.update()
        if measurement.create_solt_measurement_network(driver, port='port2', standard='load'):
            txt_status['text'] = 'Port 2 Load Standard Created'
        else:
            txt_status['text'] = 'Error: Cannot Create Port 2 Load Standard'
        pass
    
    def callback_validate_cal(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')    
        try:
            txt_status['text'] = 'Measuring Device...'
            self.mainwindow.update()
            # Receive the Calibrated Result
            result = measurement.measure_and_cal_network(driver)
            # Plot the Result in Data Page
            numpy.seterr(divide = 'ignore')
            self.fig_live.clf()
            result.plot_s_db()
            txt_status['text'] = 'Validation Complete. Results are plotted in the Data Page.'
        except:
            self.fig_live.clf()
            txt_status['text'] = 'Error: Validation Failed'
        pass
        
    def callback_apply_cal(self, event=None):
        txt_status = self.builder.get_object('label_measurement_calibration_status_text')
        # Build Measured SOLT networks based on 1 port files
        if measurement.update_solt_measurement_network_list():
            txt_status['text'] = 'Calibration Profile Generated'
        else:
            txt_status['text'] = 'Error Calibration Profile is Invalid'
        pass
    
    def callback_sweep_start(self, event=None):
        live_meas.setup_new_measurement(driver, measurement)
        plt.clf()
        
        # Plot existing lines
        i = 0
        for record in record_list.get_data_list():
            plt.plot(record.data[0], record.data[1])
            i = i+1
        plt.legend(["{}".format(j) for j in range(0, i)])
        numpy.seterr(divide = 'ignore')
        result = []
        freq = []
        s11_complex = []
        # Draw Static Part of the Plot
        plt.xlim([measurement.start_freq_ghz*1E9, measurement.stop_freq_ghz*1E9])
        plt.xlabel('Frequency (Hz)')
        plt.ylim([-20, 5])
        plt.ylabel('S11 (dB)')
        # Plot new line
        new_line, = plt.plot(freq, result)
        self.live_canvas.draw()
        
        txt_status = self.builder.get_object('label_sweep_control')
        txt_status['text'] = 'Sweeping...'
        self.mainwindow.update()
        while (True):
            seg_s11, seg_freq, seg_s11_complex = live_meas.scan_segment_s11(driver, measurement)
            
            ## Refresh Plot
            result.extend(seg_s11)
            freq.extend(seg_freq)
            s11_complex.extend(seg_s11_complex)
            new_line.set_data(freq, result)
            self.live_canvas.draw()
            
            if live_meas.is_stop_request:
                live_meas.is_stop_request = False
                break
            
            if live_meas.is_measurement_complete:
                # Restart the Measurement
                live_meas.setup_new_measurement(driver, measurement)
                # Save Records
                record_list.append_data(freq, result, new_line, s11_complex)
                                
                result = []
                freq = []
                s11_complex = []
                # Plot Existing Lines
                plt.clf()
                i = 0
                for record in record_list.get_data_list():
                    plt.plot(record.data[0], record.data[1])
                    i = i+1
                plt.legend(["{}".format(j) for j in range(0, i)]) 
                # Rescale Plot
                plt.xlim([measurement.start_freq_ghz*1E9, measurement.stop_freq_ghz*1E9])
                plt.xlabel('Frequency (Hz)')
                plt.ylim([-20, 5])
                plt.ylabel('S11 (dB)')
                # Add Current Line
                new_line, = plt.plot(freq, result)
                self.live_canvas.draw()
                ## Update Live Measurement
                last_record = record_list.get_data_list()[-1]
                txt_cal = self.builder.get_object('measurement_live_calc_txt')
                txt_cal['text'] = 'Live Calculation: Bandwidth = {:.2f}MHz Q = {:.2f}'.format(last_record.calculate_bandwidth(), last_record.calculate_q())
                ## Update Tree View
                view_records = self.builder.get_object('treeview_data_sweep_record_cal')
                for item in view_records.get_children():
                        view_records.delete(item) 
                view_records['columns'] = ('Bandwidth', 'Q')
                view_records.column('#0', width=100)
                view_records.column('Bandwidth', width=200)
                view_records.column('Q', width=200)
                view_records.heading('#0', text='ID')
                view_records.heading('Bandwidth', text='Bandwidth (MHz)')
                view_records.heading('Q', text='Q')
                for idx_record in range(0, len(record_list.record_list)):
                    view_records.insert('', tk.END, text=str(idx_record), values=("{:.2f}".format(record_list.record_list[idx_record].calculate_bandwidth()), "{:.2f}".format(record_list.record_list[idx_record].calculate_q())))
            # Update Display
            self.mainwindow.update()
        
        txt_status['text'] = 'Sweeping Stopped'
        # Plot Existing Lines
        plt.clf()
        i = 0
        for record in record_list.get_data_list():
            plt.plot(record.data[0], record.data[1])
            i = i+1
        plt.legend(["{}".format(j) for j in range(0, i)])
        # Rescale Plot
        plt.xlim([measurement.start_freq_ghz*1E9, measurement.stop_freq_ghz*1E9])
        plt.xlabel('Frequency (Hz)')
        plt.ylim([-20, 5])
        plt.ylabel('S11 (dB)')
        result = []
        freq = []
        s11_complex = []
        self.live_canvas.draw()
        ## Update Live Measurement
        last_record = record_list.get_data_list()[-1]
        txt_cal = self.builder.get_object('measurement_live_calc_txt')
        txt_cal['text'] = 'Live Calculation: Bandwidth = {:.2f}MHz Q = {:.2f}'.format(last_record.calculate_bandwidth(), last_record.calculate_q())
        ## Update Record List View
        view_records = self.builder.get_object('treeview_data_sweep_record_cal')
        for item in view_records.get_children():
                view_records.delete(item) 
        view_records['columns'] = ('Bandwidth', 'Q')
        view_records.column('#0', width=100)
        view_records.column('Bandwidth', width=200)
        view_records.column('Q', width=200)
        view_records.heading('#0', text='ID')
        view_records.heading('Bandwidth', text='Bandwidth (MHz)')
        view_records.heading('Q', text='Q')
        for idx_record in range(0, len(record_list.record_list)):
            view_records.insert('', tk.END, text=str(idx_record), values=("{:.2f}".format(record_list.record_list[idx_record].calculate_bandwidth()), "{:.2f}".format(record_list.record_list[idx_record].calculate_q())))
        pass
        
    def callback_sweep_stop(self, event=None):
        live_meas.stop_measurement(driver, measurement)
        txt_status = self.builder.get_object('label_sweep_control')
        txt_status['text'] = 'Sweeping Stopped'
        pass
    
    def callback_sweep_oneshot(self, event=None):
        live_meas.setup_new_measurement(driver, measurement)
        plt.clf()
        numpy.seterr(divide = 'ignore')
        result = []
        freq = []
        s11_complex = []
        # Plot existing lines
        i = 0
        for record in record_list.get_data_list():
            plt.plot(record.data[0], record.data[1])
            i = i+1
        plt.legend(['{}'.format(j) for j in range(0, i)])
        # Draw Static Part of the Plot
        plt.xlim([measurement.start_freq_ghz*1E9, measurement.stop_freq_ghz*1E9])
        plt.xlabel('Frequency (Hz)')
        plt.ylim([-20, 5])
        plt.ylabel('S11 (dB)')
        new_line, = plt.plot(freq, result)
        self.live_canvas.draw()
        
        txt_status = self.builder.get_object('label_sweep_control')
        txt_status['text'] = 'Sweeping...'
        self.mainwindow.update()
        
        while True:
            seg_s11, seg_freq, seg_s11_complex = live_meas.scan_segment_s11(driver, measurement)
            
            ## Refresh Plot
            result.extend(seg_s11)
            freq.extend(seg_freq)
            s11_complex.extend(seg_s11_complex)
            
            new_line.set_data(freq, result)
            self.live_canvas.draw()
            
            if live_meas.is_measurement_complete:
                break
            
            # Update Display
            self.mainwindow.update()
        
        print("Oneshot Scanning Complete")
        txt_status['text'] = 'Sweeping Stopped'
        
        # Save Newline
        record_list.append_data(freq, result, new_line, s11_complex)
        ## Update Live Measurement
        last_record = record_list.get_data_list()[-1]
        txt_cal = self.builder.get_object('measurement_live_calc_txt')
        txt_cal['text'] = 'Live Calculation: Bandwidth = {:.2f}MHz Q = {:.2f}'.format(last_record.calculate_bandwidth(), last_record.calculate_q())
        ## Update Figure
        plt.clf()
        plt.xlim([measurement.start_freq_ghz*1E9, measurement.stop_freq_ghz*1E9])
        plt.xlabel('Frequency (Hz)')
        plt.ylim([-20, 5])
        plt.ylabel('S11 (dB)')
        i = 0
        for record in record_list.get_data_list():
            plt.plot(record.data[0], record.data[1])
            i = i+1
        plt.legend(["{}".format(j) for j in range(0, i)])
        self.live_canvas.draw()
        # Update Data Records Tree View
        view_records = self.builder.get_object('treeview_data_sweep_record_cal')
        for item in view_records.get_children():
            view_records.delete(item) 
        view_records['columns'] = ('Bandwidth', 'Q')
        view_records.column('#0', width=100)
        view_records.column('Bandwidth', width=200)
        view_records.column('Q', width=200)
        view_records.heading('#0', text='ID')
        view_records.heading('Bandwidth', text='Bandwidth (MHz)')
        view_records.heading('Q', text='Q')
        for idx_record in range(0, len(record_list.record_list)):
            view_records.insert('', tk.END, text=str(idx_record), values=("{:.2f}".format(record_list.record_list[idx_record].calculate_bandwidth()), "{:.2f}".format(record_list.record_list[idx_record].calculate_q())))
        pass
    def callback_record_clear_selected(self, event):
        # Get Selected ID
        view_records = self.builder.get_object('treeview_data_sweep_record_cal')
        record_handle = view_records.focus()
        try:
            index = int(view_records.item(record_handle)["text"])
        except:
            return
        # Remove Selected Record
        record_list.remove_record(index)
        # Update List View
        view_records = self.builder.get_object('treeview_data_sweep_record_cal')
        for item in view_records.get_children():
                view_records.delete(item)
        view_records['columns'] = ('Bandwidth', 'Q')
        view_records.column('#0', width=100)
        view_records.column('Bandwidth', width=200)
        view_records.column('Q', width=200)
        view_records.heading('#0', text='ID')
        view_records.heading('Bandwidth', text='Bandwidth (MHz)')
        view_records.heading('Q', text='Q')
        for idx_record in range(0, len(record_list.record_list)):
            view_records.insert('', tk.END, text=str(idx_record), values=("{:.2f}".format(record_list.record_list[idx_record].calculate_bandwidth()), "{:.2f}".format(record_list.record_list[idx_record].calculate_q())))
        # Update Plot
        plt.clf()
        i = 0
        for record in record_list.get_data_list():
            plt.plot(record.data[0], record.data[1])
            i = i+1
        plt.legend(["{}".format(j) for j in range(0, i)])
        # Draw Static Part of the Plot
        plt.xlim([measurement.start_freq_ghz*1E9, measurement.stop_freq_ghz*1E9])
        plt.xlabel('Frequency (Hz)')
        plt.ylim([-20, 5])
        plt.ylabel('S11 (dB)')
        self.live_canvas.draw()
        
    def callback_record_clear_all(self, event):
        # Clear Record List
        record_list.remove_all_record()
        # Reset List View
        view_records = self.builder.get_object('treeview_data_sweep_record_cal')
        for item in view_records.get_children():
                view_records.delete(item) 
        view_records['columns'] = ('Bandwidth', 'Q')
        view_records.column('#0', width=100)
        view_records.column('Bandwidth', width=200)
        view_records.column('Q', width=200)
        view_records.heading('#0', text='ID')
        view_records.heading('Bandwidth', text='Bandwidth (MHz)')
        view_records.heading('Q', text='Q')
        # Reset Figure Plot
        plt.clf()
        self.live_canvas.draw()
        # Draw Static Part of the Plot
        plt.xlim([measurement.start_freq_ghz*1E9, measurement.stop_freq_ghz*1E9])
        plt.xlabel('Frequency (Hz)')
        plt.ylim([-20, 5])
        plt.ylabel('S11 (dB)')

    def callback_double_click_list_view(self, event):
        # Get Current Selection Handle
        view_records = self.builder.get_object('treeview_data_sweep_record_cal')
        record_handle = view_records.focus()
        try:
            bw = float(simpledialog.askstring(title="Debug", prompt="Corrected Bandwidth(MHz):"))
            q = float(simpledialog.askstring(title="Debug", prompt="Corrected Q:"))
            index = int(view_records.item(record_handle)["text"])
            view_records.item(record_handle, text="{}".format(index), values=("{:.2f}".format(bw), "{:.2f}".format(q)))
        except:
            return
        self.mainwindow.update()
        
    def callback_destroy(self, event):
        if event.widget is self.mainwindow:
            driver.close()
            print("Exiting")
            exit(0)
        pass

if __name__ == '__main__':
    # Class Init
    app = SavermainappApp()
    driver = pocketvna.Driver()
    measurement = meas.Measurement()
    live_meas = meas.LiveMeasurement(measurement)
    record_list = records.RecordList()
    # Application Main Loop
    app.run()
