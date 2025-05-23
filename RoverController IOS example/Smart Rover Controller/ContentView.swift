//  ContentView.swift
//  Smart Rover Controller
//
//  Created by Moutaz Baaj on 06.05.25.
//

import SwiftUI

struct ContentView: View {
    
    @State private var autoMode: Bool = false
    @State private var led: Bool = true
    
    var body: some View {
        GeometryReader { geo in
            let w = geo.size.width
            let h = geo.size.height
            
            NavigationStack {
                VStack(spacing: 20) {
                    
                    HStack {
                        
                        // Retro status screen
                        Text(viewModel.shared.statusMessage)
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(.green)
                            .padding(8)
                            .background(Color.black)
                            .cornerRadius(12)
                            .overlay(
                                RoundedRectangle(cornerRadius: 12)
                                    .stroke(Color.green.opacity(0.6), lineWidth: 1)
                            )
                            .shadow(radius: 5)
                            .lineLimit(2)
                            .minimumScaleFactor(0.5)
                        
                        Spacer()
                    }
                    
                    
                    // Control panel
                    HStack(spacing: 16) {
                        
                        // Left panel
                        ControlCard(title: "Steering", width: w / 3.3) {
                            VStack(spacing: 15) {
                                HStack {
                                    viewModel.shared.holdButton("←", command: "left")
                                    Button {
                                        viewModel.shared.sendCommand("stop")
                                    } label: {
                                        Image(systemName: "stop.fill")
                                            .foregroundStyle(.red)
                                        
                                    }
                                    viewModel.shared.holdButton("→", command: "right")
                                }
                            }
                        }
                        
                        // center panel
                        Spacer()
                        ControlCard(title: "Steering", width: w / 3.3) {
                            VStack(spacing: 15) {
                                HStack {
                                    viewModel.shared.holdButton("◀︎", command: "servo_left")
                                    Button("⦿") { viewModel.shared.sendCommand("center_servo") }
                                    viewModel.shared.holdButton("▶︎", command: "servo_right")
                                }
                            }
                        }
                        
                        Spacer()
                        
                        // Right panel
                        ControlCard(title: "Drive", width: w / 3.3) {
                            VStack(spacing: 15) {
                                viewModel.shared.holdButton("↑", command: "forward")
                                Button {
                                    viewModel.shared.sendCommand("stop")
                                } label: {
                                    Image(systemName: "stop.fill")
                                        .foregroundStyle(.red)
                                }          
                                viewModel.shared.holdButton("↓", command: "backward")
                            }
                        }
                    }
                    .font(.title2)
                    .buttonStyle(.borderedProminent)
                    .controlSize(.large)
                    .padding(.horizontal)
                }
                .toolbar {
                    Menu {
                        Button {
                            autoMode.toggle()
                            viewModel.shared.sendCommand("toggle_auto")
                        } label: {
                            Label(autoMode ? "Disable Auto Mode" : "Enable Auto Mode", systemImage: "car.top.radiowaves.front")
                        }
                        
                        Button {
                            led.toggle()
                            viewModel.shared.sendCommand("toggle_led")
                        } label: {
                            Label(led ? "Turn Off LED" : "Turn On LED", systemImage: "lightbulb")
                        }
                        VStack(spacing: 8) {
                            Text("Speed")
                                .font(.headline)
                            HStack {
                                Button {
                                    viewModel.shared.sendCommand("speeddown")
                                } label: {
                                    Label("Slow", systemImage: "minus.circle")
                                }
                                Button {
                                    viewModel.shared.sendCommand("speedup")
                                } label: {
                                    Label("Fast", systemImage: "plus.circle")
                                }
                            }
                        }
                        
                    } label: {
                        Label("Menu", systemImage: "slider.horizontal.3")
                    }
                }
                .navigationTitle("Smart Rover")
                .navigationBarTitleDisplayMode(.inline)
                .frame(width: w, height: h)
                
            }
            
        }
        .onDisappear { viewModel.shared.sendCommand("stop") }
    }
}

#Preview {
    ContentView()
}
