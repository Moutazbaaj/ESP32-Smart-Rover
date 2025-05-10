//
//  ControllerViewModel.swift
//  Smart Rover Controller
//
//  Created by Moutaz Baaj on 07.05.25.
//

import Foundation
import SwiftUICore
import SwiftUI

class ControllerViewModel: ObservableObject {
    
    static let shared = ControllerViewModel()
    
    @Published var statusMessage = "Smart Rover Ready"
    private var activeCommands = Set<String>()
    private var lastSentCommand = ""
    private var comboWasActive = false
    private var holdTimer: Timer?
    private let baseURL = "http://192.168.4.1"
    

    
    func holdButton(_ label: String, command: String) -> some View {
        Text(label)
            .frame(width: 60, height: 60)
            .background(Color.blue)
            .foregroundColor(.white)
            .clipShape(Circle())
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { _ in self.handlePress(command: command) }
                    .onEnded { _ in self.handleRelease(command: command) }
            )
    }
    
    private func handlePress(command: String) {
        guard isMovementCommand(command) else {
            // Handle non-movement commands with hold timer
            if holdTimer == nil {
                sendCommand(command)
                startHoldTimer(command)
            }
            return
        }
        
        let previousCount = activeCommands.count
        activeCommands.insert(command)
        
        // Only send command if state changed
        if activeCommands.count != previousCount {
            sendCombinedCommand()
        }
    }
    
    private func handleRelease(command: String) {
        guard isMovementCommand(command) else {
            stopHoldTimer()
            return
        }
        
        let previousCount = activeCommands.count
        activeCommands.remove(command)
        
        // Only send command if state changed
        if activeCommands.count != previousCount {
            if activeCommands.isEmpty {
                sendCommand("stop")
                lastSentCommand = ""
                comboWasActive = false
            } else {
                sendCombinedCommand()
            }
        }
    }
    
    private func isMovementCommand(_ command: String) -> Bool {
        return ["forward", "backward", "left", "right"].contains(command)
    }
    
    private func isComboCommand(_ command: String) -> Bool {
        let comboCommands = ["forward_left", "forward_right", "backward_left", "backward_right"]
        return comboCommands.contains(command)
    }
    
    private func buildCurrentCommand() -> String {
        if activeCommands.contains("forward") {
            if activeCommands.contains("left") { return "forward_left" }
            if activeCommands.contains("right") { return "forward_right" }
            return "forward"
        }
        if activeCommands.contains("backward") {
            if activeCommands.contains("left") { return "backward_left" }
            if activeCommands.contains("right") { return "backward_right" }
            return "backward"
        }
        if activeCommands.contains("left") { return "left" }
        if activeCommands.contains("right") { return "right" }
        return "stop"
    }
    
    private func sendCombinedCommand() {
        let newCommand = buildCurrentCommand()
        
        if newCommand != lastSentCommand {
            // Handle command transitions
            if isComboCommand(lastSentCommand) && !isComboCommand(newCommand) && !activeCommands.isEmpty {
                sendCommand("stop") // Send stop first for clean transition
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.05) {
                    self.sendCommand(newCommand)
                }
            } else {
                sendCommand(newCommand)
            }
            
            lastSentCommand = newCommand
            comboWasActive = isComboCommand(newCommand)
        }
    }
    
    private func startHoldTimer(_ command: String) {
        guard holdTimer == nil else { return }
        holdTimer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { _ in
            self.sendCommand(command)
        }
    }
    
    private func stopHoldTimer() {
        holdTimer?.invalidate()
        holdTimer = nil
    }
    
    func sendCommand(_ endpoint: String) {
        guard let url = URL(string: "\(baseURL)/\(endpoint)") else { return }
        URLSession.shared.dataTask(with: url) { _, response, _ in
            if let httpRes = response as? HTTPURLResponse {
                DispatchQueue.main.async {
                    self.statusMessage = "Sent \(endpoint.uppercased()) (\(httpRes.statusCode))"
                }
            }
        }.resume()
    }
    
}
