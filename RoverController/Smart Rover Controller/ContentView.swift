//  ContentView.swift
//  Smart Rover Controller
//
//  Created by Moutaz Baaj on 06.05.25.
//

import SwiftUI

struct ContentView: View {
    
    @StateObject private var viewModel = ControllerViewModel.shared  
    
    var body: some View {
        GeometryReader { geo in
            let w = geo.size.width
            let h = geo.size.height
            VStack {
                Text(viewModel.statusMessage)
                    .font(.headline)
                    .padding()
                
                HStack(spacing: 20) {
                    
                    // left:
                    VStack(spacing: 15) {
                        Spacer()
                        
                        HStack {
                            viewModel.holdButton("←", command: "left")
                            Button("🛑") { viewModel.sendCommand("stop") }
                            viewModel.holdButton("→", command: "right")
                        }
                        
                        Spacer()
                        
                        Button("Auto Mode") { viewModel.sendCommand("toggle_auto") }
                    }
                    .frame(width: w * 0.3)
                    
                    // center panel: Servo & Other Controls
                    VStack(spacing: 15) {
                        Text("Servo")
                        HStack {
                            viewModel.holdButton("◀︎", command: "servo_left")
                            Button("⦿") { viewModel.sendCommand("center_servo") }
                            viewModel.holdButton("▶︎", command:"servo_right") 
                        }
                        
                        Text("Speed")
                        HStack {
                            Button("➖") { viewModel.sendCommand("speeddown") }
                            Button("➕") { viewModel.sendCommand("speedup") }
                        }
                        
                    }
                    .frame(width: w * 0.3)
                    
                    // right:
                    VStack(spacing: 10) {
                        Spacer()
                        viewModel.holdButton("↑", command: "forward")
                        Button("🛑") { viewModel.sendCommand("stop")
                        }
                        viewModel.holdButton("↓", command: "backward")
                        Spacer()
                        Button("Toggle LED") { viewModel.sendCommand("toggle_led") }
                        
                    }
                    
                    .frame(width: w * 0.3)
                }
                .font(.title)
                .buttonStyle(.borderedProminent)
                .controlSize(.large)
            }
            .frame(width: w, height: h)
        }
        .onDisappear { viewModel.sendCommand("stop")}
    }
    
    
}

#Preview {
    ContentView()
}
