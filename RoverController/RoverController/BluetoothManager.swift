//
//  BluetoothManager.swift
//  RoverController
//
//  Created by Moutaz Baaj on 27.03.25.
//

import CoreBluetooth

class BluetoothManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private var centralManager: CBCentralManager!
    private var roverPeripheral: CBPeripheral?
    private var txCharacteristic: CBCharacteristic?
    
    @Published var autoMode = false

    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            central.scanForPeripherals(withServices: nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        if peripheral.name == "ESP32_Rover" {
            roverPeripheral = peripheral
            roverPeripheral?.delegate = self
            centralManager.stopScan()
            centralManager.connect(peripheral)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices(nil)
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let service = peripheral.services?.first {
            peripheral.discoverCharacteristics(nil, for: service)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let characteristic = service.characteristics?.first {
            txCharacteristic = characteristic
        }
    }
    
    func sendCommand(_ command: String) {
        guard let peripheral = roverPeripheral, let characteristic = txCharacteristic else { return }
        let data = command.data(using: .utf8)
        peripheral.writeValue(data!, for: characteristic, type: .withResponse)
    }
}
