import SwiftUI
import WebKit

struct ContentView: View {
    @State private var speed: Int = 5
    @State private var autoMode = false
    @State private var isConnected = false
    @State private var showWebView = false
    @State private var activeCommand: String? = nil
    private let timer = Timer.publish(every: 0.1, on: .main, in: .common).autoconnect()
    
    let roverIP = "192.168.4.1"
    
    var body: some View {
        VStack {
            Text("ESP32 Rover Control")
                .font(.title)
                .padding()
            
            if isConnected {
                joystickController
                speedControl
                autoModeToggle
            } else {
                connectButton
            }
        }
        .sheet(isPresented: $showWebView) {
            WebView(url: URL(string: "http://\(roverIP)")!)
        }
        .onReceive(timer) { _ in
            handleCommandSending()
        }
    }
    
    // MARK: - Command Handling
    private func handleCommandSending() {
        if autoMode {
            if activeCommand != "auto" {
                activeCommand = "auto"
                sendCommand("auto")
            }
        } else {
            var command = activeCommand ?? "stop"
            sendCommand(command)
        }
    }
    private func sendCommand(_ command: String) {
        guard let url = URL(string: "http://\(roverIP)/\(command)") else { return }
        
        URLSession.shared.dataTask(with: url) { _, _, _ in }.resume()
    }
    
    // MARK: - UI Components
    var joystickController: some View {
        VStack(spacing: 20) {
            // Forward Button
            DirectionalButton(
                direction: "forward",
                systemImage: "arrow.up",
                activeCommand: $activeCommand,
                isDisabled: autoMode,
                onRelease: { activeCommand = nil }
            )
            


            HStack(spacing: 20) {
                // Left Button
                DirectionalButton(
                    direction: "left",
                    systemImage: "arrow.left",
                    activeCommand: $activeCommand,
                    isDisabled: autoMode,
                    onRelease: { 
                        if activeCommand == "forwardLeft" {
                            activeCommand = "forward"
                        } else if activeCommand == "backwardLeft" {
                            activeCommand = "backward"
                        } else {
                            activeCommand = nil
                        }
                    }
                )
                
                // Stop Button
                Button(action: {
                    activeCommand = nil
                    sendCommand("stop")
                }) {
                    Image(systemName: "stop.fill")
                        .frame(width: 80, height: 80)
                }
                .buttonStyle(JoystickButtonStyle(backgroundColor: .red))
                .disabled(autoMode)                .buttonStyle(JoystickButtonStyle(backgroundColor: .red))
                .disabled(autoMode)
                
                // Right Button

                DirectionalButton(
                    direction: "right",
                    systemImage: "arrow.right",
                    activeCommand: $activeCommand,
                    isDisabled: autoMode,
                    onRelease: { 
                        if activeCommand == "forwardRight" {
                            activeCommand = "forward"
                        } else if activeCommand == "backwardRight" {
                            activeCommand = "backward"
                        } else {
                            activeCommand = nil
                        }
                    }
                )
            }
            
            // Backward Button
            DirectionalButton(
                direction: "backward",
                systemImage: "arrow.down",
                activeCommand: $activeCommand,
                isDisabled: autoMode,
                onRelease: { activeCommand = nil }
            )
        }
        .padding()
        .opacity(autoMode ? 0.6 : 1.0)
    }
    
    var speedControl: some View {
        VStack {
            Text("Speed: \(speed)")
            Slider(value: Binding(
                get: { Double(speed) },
                set: { 
                    speed = Int($0)
                    if !autoMode {
                        sendCommand("speed?val=\(speed)")
                    }
                }
            ), in: 0...9, step: 1)
            .disabled(autoMode)
            .padding()
        }
    }
    
    var autoModeToggle: some View {
        Toggle("Autonomous Mode", isOn: $autoMode)
            .onChange(of: autoMode) {_, newValue in
                if newValue {
                    // When enabling auto mode
                    activeCommand = "auto"
                    sendCommand("auto")
                } else {
                    // When disabling auto mode
                    activeCommand = nil
                    sendCommand("stop")
                }
            }
            .padding()
    }
    
    var connectButton: some View {
        VStack {
            Button("Connect to Rover") {
                isConnected = true
            }
            .padding()
            .background(Color.blue)
            .foregroundColor(.white)
            .cornerRadius(8)
            
            Button("Open Web Interface") {
                showWebView = true
            }
            .padding()
        }
    }
}

// MARK: - Custom Components
struct DirectionalButton: View {
    let direction: String
    let systemImage: String
    @Binding var activeCommand: String?
    var isDisabled: Bool
    var onRelease: () -> Void

    var body: some View {
        Button(action: {
            if !isDisabled {
                if direction == "left" || direction == "right" {
                    // Append turning direction to movement
                    if activeCommand == "forward" {
                        activeCommand = "forward\(direction.capitalized)"
                    } else if activeCommand == "backward" {
                        activeCommand = "backward\(direction.capitalized)"
                    } else {
                        activeCommand = direction
                    }
                } else {
                    activeCommand = direction
                }
            }
        }) {
            Image(systemName: systemImage)
                .frame(width: 80, height: 80)
        }
        .buttonStyle(JoystickButtonStyle(
            backgroundColor: isDisabled ? .gray : .blue
        ))
        .disabled(isDisabled)
        .simultaneousGesture(
            DragGesture(minimumDistance: 0)
                .onEnded { _ in
                    if !isDisabled {
                        onRelease()
                    }
                }
        )
    }
}
struct JoystickButtonStyle: ButtonStyle {
    var backgroundColor: Color
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.title)
            .foregroundColor(.white)
            .background(
                Circle()
                    .fill(backgroundColor)
                    .shadow(radius: configuration.isPressed ? 2 : 5)
                    .scaleEffect(configuration.isPressed ? 0.9 : 1.0)
            )
            .animation(.easeOut(duration: 0.1), value: configuration.isPressed)
    }
}

struct WebView: UIViewRepresentable {
    let url: URL
    
    func makeUIView(context: Context) -> WKWebView {
        return WKWebView()
    }
    
    func updateUIView(_ uiView: WKWebView, context: Context) {
        let request = URLRequest(url: url)
        uiView.load(request)
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
