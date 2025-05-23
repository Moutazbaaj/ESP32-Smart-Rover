//
//  ControlCard.swift
//  Smart Rover Controller
//
//  Created by Moutaz Baaj on 13.05.25.
//

import SwiftUICore


struct ControlCard<Content: View>: View {
    var title: String
    var width: CGFloat
    var content: () -> Content
    
    var body: some View {
        VStack(spacing: 10) {
            Text(title)
                .font(.headline)
                .padding(.top, 8)
            content()
                .padding()
        }
        .frame(width: width)
        .background(Color(.systemGray6))
        .cornerRadius(16)
        .shadow(radius: 3)
    }
}
