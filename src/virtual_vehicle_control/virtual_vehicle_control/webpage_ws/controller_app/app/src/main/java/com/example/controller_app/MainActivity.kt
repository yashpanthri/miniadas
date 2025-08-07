package com.example.controller_app

import android.annotation.SuppressLint
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.MotionEvent
import android.widget.Button
import android.widget.TextView
import android.widget.ToggleButton
import androidx.activity.ComponentActivity
import com.google.gson.Gson
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener

class MainActivity : ComponentActivity() {

    // UI Elements - Declared here for easy access in callbacks
    private lateinit var heading: TextView
    private lateinit var gearToggle: ToggleButton

    /** ─────────── ROSBRIDGE CLIENT ─────────── */
    // Made it an inner class to access MainActivity's members like `runOnUiThread` and UI elements.
    private inner class RosBridgeClient(
        // Using the special IP for the Android Emulator to connect to the host machine.
        private val url: String = "ws://10.0.2.2:9090"
    ) : WebSocketListener() {

        private val gson = Gson()
        private val client = OkHttpClient()
        private lateinit var ws: WebSocket

        fun connect() {
            val request = Request.Builder().url(url).build()
            ws = client.newWebSocket(request, this)
        }

        fun close() {
            if (::ws.isInitialized) {
                ws.close(1000, "Activity Destroyed")
            }
        }

        override fun onOpen(webSocket: WebSocket, response: Response) {
            super.onOpen(webSocket, response)
            runOnUiThread {
                heading.text = "Status: Connected"
            }
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            super.onFailure(webSocket, t, response)
            Log.e("RosBridgeClient", "Connection Failed!", t)
            runOnUiThread {
                heading.text = "Failed: ${t.message}"
            }
        }


        fun publishCtrl(
            throttle: Float,
            steer: Float,
            brake: Float,
            reverse: Boolean,
            gear: Int
        ) {
            if (!::ws.isInitialized) return
            val msg = mapOf(
                "op" to "publish",
                "topic" to "/carla/hero/vehicle_control_cmd_manual",
                "msg" to mapOf(
                    "throttle" to throttle,
                    "steer" to steer,
                    "brake" to brake,
                    "hand_brake" to false,
                    "reverse" to reverse,
                    "gear" to gear,
                    "manual_gear_shift" to false
                )
            )
            ws.send(gson.toJson(msg))
        }
    }
    /** ───────────────────────────────────────── */

    // App-wide state
    private val ros = RosBridgeClient()
    private var gear = 1          // 1 = F, −1 = R
    private var steer = 0f
    private var throttle = 0f
    private var brake = 0f

    // --- NEW: Handler and state for 20Hz publishing ---
    private val handler = Handler(Looper.getMainLooper())
    private var activeButtons = 0 // Counter for how many buttons are being pressed

    // --- NEW: This Runnable is the core of the 20Hz loop ---
    private val publisherRunnable = object : Runnable {
        override fun run() {
            publishCurrent()
            // Reschedule itself to run again in 50ms (1000ms / 20Hz = 50ms)
            handler.postDelayed(this, 50)
        }
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.main_layout)

        ros.connect()

        heading = findViewById(R.id.heading)
        val leftButton: Button = findViewById(R.id.left_button)
        val rightButton: Button = findViewById(R.id.right_button)
        val throttleButton: Button = findViewById(R.id.throttle_button)
        val brakeButton: Button = findViewById(R.id.brake_button)
        gearToggle = findViewById(R.id.gear_toggle)

        heading.text = "Status: Connecting..."

        // --- UPDATED LISTENERS to use the new centralized handler ---
        leftButton.setOnTouchListener { _, ev ->
            handleButtonPress(ev.action) { steer = if (it) -1f else 0f }
            true
        }
        rightButton.setOnTouchListener { _, ev ->
            handleButtonPress(ev.action) { steer = if (it) 1f else 0f }
            true
        }
        throttleButton.setOnTouchListener { _, ev ->
            handleButtonPress(ev.action) {
                throttle = if (it) 0.5f else 0f
                if (it) brake = 0f // Can't press throttle and brake
            }
            true
        }
        brakeButton.setOnTouchListener { _, ev ->
            handleButtonPress(ev.action) {
                brake = if (it) 1f else 0f
                if (it) throttle = 0f // Can't press brake and throttle
            }
            true
        }

        gearToggle.setOnCheckedChangeListener { _, isChecked ->
            gear = if (isChecked) -1 else 1
            heading.text = if (isChecked) "Gear: Reverse" else "Gear: Forward"
            // A gear change is a single event, so we publish it once immediately.
            publishCurrent()
        }
    }

    /**
     * NEW: A helper function to manage the publishing loop. It starts the loop
     * when the first button is pressed and stops it when the last one is released.
     * @param action The MotionEvent action (e.g., ACTION_DOWN).
     * @param updateState A function that updates the relevant control variable (steer, throttle, etc.).
     */
    private fun handleButtonPress(action: Int, updateState: (isPressed: Boolean) -> Unit) {
        when (action) {
            MotionEvent.ACTION_DOWN -> {
                activeButtons++
                updateState(true)
                // If this is the very first button pressed, start the 20Hz publisher.
                if (activeButtons == 1) {
                    handler.post(publisherRunnable)
                }
            }
            MotionEvent.ACTION_UP -> {
                activeButtons--
                updateState(false)
                // If this was the last button being held down, stop the publisher.
                if (activeButtons == 0) {
                    handler.removeCallbacks(publisherRunnable)
                }
                // Send one final state update on release to ensure it's neutral.
                publishCurrent()
            }
        }
    }

    /** Publish frame with current state */
    private fun publishCurrent() = ros.publishCtrl(
        throttle = throttle,
        steer = steer,
        brake = brake,
        reverse = gear == -1,
        gear = gear
    )

    /* Safety: stop vehicle if activity leaves foreground */
    override fun onPause() {
        super.onPause()
        // --- UPDATED: Also stop the handler when the app is paused ---
        handler.removeCallbacks(publisherRunnable)
        activeButtons = 0
        // Reset state variables
        throttle = 0f
        steer = 0f
        brake = 1f
        // Send a full brake command
        ros.publishCtrl(0f, 0f, 1f, reverse = gear == -1, gear = gear)
    }

    /* Gracefully close the connection when the app is closed */
    override fun onDestroy() {
        super.onDestroy()
        ros.close()
    }
}