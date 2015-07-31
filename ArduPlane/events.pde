// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void failsafe_short_on_event(enum failsafe_state fstype)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = fstype;
    failsafe.ch3_timer_ms = millis();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event on, "));
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
		// only handle failsafe if enabled
		// short failsafe is triggered only on throlle
		if( g.failsafe_manual_mode & FS_MODE_THR ) {
			// change to RTL
			failsafe.saved_mode = control_mode;
			failsafe.saved_mode_set = 1;
			set_mode(RTL);
		}
        break;

    case AUTO:
	case GUIDED:
    case LOITER:
	case CIRCLE:
    case RTL:
		// do nothing on short failsafe in automatic modes and CIRCLE
		// all these modes are safe in case of short control loss
		break;

	default:
		break;
    }
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void failsafe_long_on_event(enum failsafe_state fstype)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Long event on, "));
    //  If the GCS is locked up we allow control to revert to RC
    hal.rcin->clear_overrides();
    failsafe.state = fstype;
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
    case CIRCLE:
		// handle failsafe if enabled
		if( ( g.failsafe_manual_mode & FS_MODE_THR ) && fstype == FAILSAFE_LONG
			|| ( g.failsafe_manual_mode & FS_MODE_GCS ) && fstype == FAILSAFE_GCS )
		{
			set_mode(RTL);
        }
        break;

    case AUTO:
	case GUIDED:
    case LOITER:
		// handle failsafe if enabled
		if( ( g.failsafe_auto_mode & FS_MODE_THR ) && fstype == FAILSAFE_LONG
			|| ( g.failsafe_auto_mode & FS_MODE_GCS ) && fstype == FAILSAFE_GCS )
		{
			set_mode(RTL);
        }
        break;

    case RTL:
    default:
        break;
    }
    if (fstype == FAILSAFE_GCS) {
        gcs_send_text_P(SEVERITY_HIGH, PSTR("No GCS heartbeat."));
    }
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void failsafe_short_off_event()
{
    // We're back in radio contact
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event off"));
    failsafe.state = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == RTL && failsafe.saved_mode_set) {
        failsafe.saved_mode_set = 0;
        set_mode(failsafe.saved_mode);
    }
}

void low_battery_event(void)
{
    if (failsafe.low_battery) {
        return;
    }
    gcs_send_text_fmt(PSTR("Low Battery %.2fV Used %.0f mAh"),
                      battery.voltage(), battery.current_total_mah());
    set_mode(RTL);
    aparm.throttle_cruise.load();
    failsafe.low_battery = true;
    AP_Notify::flags.failsafe_battery = true;
}

static void update_events(void)
{
    ServoRelayEvents.update_events();
}
