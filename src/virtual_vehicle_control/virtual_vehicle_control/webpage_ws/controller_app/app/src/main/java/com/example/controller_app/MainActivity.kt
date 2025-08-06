package com.example.controller_app

import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import android.widget.ToggleButton
import androidx.activity.ComponentActivity

//import androidx.activity.ComponentActivity

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
//        enableEdgeToEdge()
//        setContent {
//            Controller_appTheme {
//                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
//                    MyAppUI(
////                        name = "controller app",
//                        modifier = Modifier.padding(innerPadding)
//                    )
//                }
//            }
//        }
        setContentView(R.layout.main_layout)  // ✅ Use your XML layout

        // Connect buttons
        val heading: TextView = findViewById(R.id.heading)
        val leftButton: Button = findViewById(R.id.left_button)
        val rightButton: Button = findViewById(R.id.right_button)
        val throttleButton: Button = findViewById(R.id.throttle_button)
        val brakeButton: Button = findViewById(R.id.brake_button)
        val gearToggle: ToggleButton = findViewById(R.id.gear_toggle)

        var speed = 0

        // Left Button
        leftButton.setOnClickListener {
            heading.text = "Turning Left"
        }

        // Right Button
        rightButton.setOnClickListener {
            heading.text = "Turning Right"
        }

        // Throttle
        throttleButton.setOnClickListener {
            speed += 10
            heading.text = "Speed: $speed km/h"
        }

        // Brake
        brakeButton.setOnClickListener {
            if (speed > 0) speed -= 10
            heading.text = "Brake applied. Speed: $speed km/h"
        }

        // Gear Toggle
        gearToggle.setOnCheckedChangeListener { _, isChecked ->
            heading.text = if (isChecked) "Gear: Reverse" else "Gear: Forward"
        }
    }
}





//@Composable
//fun MyAppUI(modifier: Modifier = Modifier) {
//    var text by remember { mutableStateOf("Hello") }
//    var toggleState by remember { mutableStateOf(false) }
//    var speed by remember { mutableStateOf(0) } // For throttle & brake
//
//    Column(
//        modifier = modifier
//            .fillMaxSize()
//            .padding(16.dp),
//        verticalArrangement = Arrangement.Center,
//        horizontalAlignment = Alignment.CenterHorizontally
//    ) {
//        // Button 1 (Left)
//        Button(onClick = { text = "Turning Left" }, modifier = Modifier.fillMaxWidth()) {
//            Text("Left")
//        }
//        Spacer(modifier = Modifier.height(8.dp))
//
//        // Button 2 (Right)
//        Button(onClick = { text = "Turning Right" }, modifier = Modifier.fillMaxWidth()) {
//            Text("Right")
//        }
//        Spacer(modifier = Modifier.height(8.dp))
//
//        // Button 3 (Throttle)
//        Button(onClick = {
//            speed += 10
//            text = "Throttle: $speed km/h"
//        }, modifier = Modifier.fillMaxWidth()) {
//            Text("Throttle")
//        }
//        Spacer(modifier = Modifier.height(8.dp))
//
//        // Button 4 (Brake)
//        Button(onClick = {
//            if (speed > 0) speed -= 10
//            text = "Brake: $speed km/h"
//        }, modifier = Modifier.fillMaxWidth()) {
//            Text("Brake")
//        }
//        Spacer(modifier = Modifier.height(12.dp))
//
//        // Toggle Button (Gear)
//        Button(onClick = { toggleState = !toggleState }, modifier = Modifier.fillMaxWidth()) {
//            Text(if (toggleState) "Gear: Reverse" else "Gear: Forward")
//        }
//        Spacer(modifier = Modifier.height(16.dp))
//
//        // TextView equivalent
//        Text(text = text, style = MaterialTheme.typography.bodyLarge)
//    }
//}
////fun Greeting(name: String, modifier: Modifier = Modifier) {
////    Text(
////        text = "Hello $name!",
////        modifier = modifier
////    )
////}
//
////@Preview(showBackground = true)
////@Composable
////fun GreetingPreview() {
////    Controller_appTheme {
////        Greeting("Android")
////    }
////}