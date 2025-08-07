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
import androidx.appcompat.app.AppCompatActivity
import com.google.gson.Gson
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener

class MainActivity : AppCompatActivity() {

    // UI Elements
    private lateinit var heading: TextView
    private lateinit var gearToggle: ToggleButton
    private lateinit var statusText: TextView

    /** ─────────── ROSBRIDGE CLIENT ─────────── */
    private inner class RosBridgeClient(
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
                heading.text = "CARLA Ego Vehicle Controller"
                updateStatusText()
            }
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            super.onFailure(webSocket, t, response)
            Log.e("RosBridgeClient", "Connection Failed!", t)
            runOnUiThread {
                heading.text = "Connection Failed"
                statusText.text = "---"
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
                    "manual_gear_shift" to true
                )
            )
            ws.send(gson.toJson(msg))
        }
    }
    /** ───────────────────────────────────────── */

    // App-wide state
    private val ros = RosBridgeClient()
    private var gear = 1
    private var steer = 0f
    private var throttle = 0f
    private var brake = 0f

    private val handler = Handler(Looper.getMainLooper())
    private var activeButtons = 0

    private val publisherRunnable = object : Runnable {
        override fun run() {
            publishCurrent()
            handler.postDelayed(this, 50)
        }
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.main_layout)

        ros.connect()

        heading = findViewById(R.id.heading)
        statusText = findViewById(R.id.status_text)
        val leftButton: Button = findViewById(R.id.left_button)
        val rightButton: Button = findViewById(R.id.right_button)
        val throttleButton: Button = findViewById(R.id.throttle_button)
        val brakeButton: Button = findViewById(R.id.brake_button)
        gearToggle = findViewById(R.id.gear_toggle)

        heading.text = "Connecting to CARLA..."
        updateStatusText()

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
                if (it) brake = 0f
            }
            true
        }
        brakeButton.setOnTouchListener { _, ev ->
            handleButtonPress(ev.action) {
                brake = if (it) 1f else 0f
                if (it) throttle = 0f
            }
            true
        }

        gearToggle.setOnCheckedChangeListener { _, isChecked ->
            gear = if (isChecked) -1 else 1
            // When gear changes, ensure brake is set correctly for the new idle state
            if (activeButtons == 0) {
                brake = if (gear == -1) 0.01f else 0f
            }
            updateStatusText()
            publishCurrent()
        }
    }

    private fun handleButtonPress(action: Int, updateState: (isPressed: Boolean) -> Unit) {
        when (action) {
            MotionEvent.ACTION_DOWN -> {
                // Clear any holding brake when starting a new action
                if (brake < 0.1f) brake = 0f
                activeButtons++
                updateState(true)
                if (activeButtons == 1) {
                    handler.post(publisherRunnable)
                }
            }
            MotionEvent.ACTION_UP -> {
                activeButtons--
                updateState(false)
                if (activeButtons == 0) {
                    handler.removeCallbacks(publisherRunnable)
                    // --- THIS IS THE FIX ---
                    // When idle, apply a tiny holding brake in reverse to prevent
                    // shifting to neutral, and ensure true coasting in forward.
                    if (gear == -1) {
                        brake = 0.01f
                    } else {
                        brake = 0f
                    }
                }
                publishCurrent()
            }
        }
        updateStatusText()
    }

    private fun updateStatusText() {
        val gearStatus = if (gear == 1) "Forward" else "Reverse"
        val actionStatus = when {
            brake > 0.1f -> "Braking"
            throttle > 0f -> "Accelerating"
            steer < 0f -> "Turning Left"
            steer > 0f -> "Turning Right"
            else -> "Coasting" // Changed from "Idle" for clarity
        }
        statusText.text = "Gear: $gearStatus  |  Action: $actionStatus"
    }

    private fun publishCurrent() = ros.publishCtrl(
        throttle = throttle,
        steer = steer,
        brake = brake,
        reverse = gear == -1,
        gear = gear
    )

    override fun onPause() {
        super.onPause()
        handler.removeCallbacks(publisherRunnable)
        activeButtons = 0
        throttle = 0f
        steer = 0f
        brake = 1f
        ros.publishCtrl(0f, 0f, 1f, reverse = gear == -1, gear = gear)
        updateStatusText()
    }

    override fun onDestroy() {
        super.onDestroy()
        ros.close()
    }
}
