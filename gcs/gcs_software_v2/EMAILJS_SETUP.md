# EmailJS Landing Notification Setup Guide

## Overview

This Ground Control Station (GCS) uses **EmailJS** to automatically send landing notification emails when the CanSat touches down (flight_state === 7). EmailJS is a client-side email service that sends emails directly from the browser without needing a backend server.

## 📋 Prerequisites

- Email account (Gmail, Outlook, Yahoo, etc.)
- EmailJS account (free tier available)
- CanSat GCS running in browser

## 🚀 Quick Setup (5 minutes)

### Step 1: Create EmailJS Account

1. Go to https://www.emailjs.com/
2. Click **Sign Up** (free tier: 200 emails/month)
3. Verify your email address
4. Log in to the EmailJS dashboard

### Step 2: Add Email Service

1. In EmailJS dashboard, go to **Email Services**
2. Click **Add New Service**
3. Select your email provider:
   - **Gmail** (recommended for testing)
   - **Outlook/Hotmail**
   - **Yahoo**
   - Or use custom SMTP
4. Follow the provider-specific setup:

#### For Gmail:
   - Click "Connect Account"
   - Sign in with your Gmail account
   - Allow EmailJS to send emails on your behalf
   - Name your service (e.g., "CanSat Notifications")
   - Save Service (note the **Service ID** - you'll need this!)

#### For Custom SMTP:
   - Enter SMTP server details
   - Use app-specific password (not your regular password)
   - Test connection
   - Save and note the **Service ID**

### Step 3: Create Email Template

1. In EmailJS dashboard, go to **Email Templates**
2. Click **Create New Template**
3. Use this template configuration:

**Template Name:** `cansat_landing_notification`

**Template Content:**
```html
Subject: 🛬 CanSat Landing Detected - {{team_id}}

<!DOCTYPE html>
<html>
<head>
    <style>
        body { font-family: Arial, sans-serif; line-height: 1.6; color: #333; }
        .container { max-width: 600px; margin: 0 auto; padding: 20px; }
        .header { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 30px; text-align: center; border-radius: 10px 10px 0 0; }
        .content { background: #f4f4f4; padding: 30px; border-radius: 0 0 10px 10px; }
        .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin: 20px 0; }
        .data-item { background: white; padding: 15px; border-radius: 5px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .data-label { font-size: 12px; color: #666; text-transform: uppercase; }
        .data-value { font-size: 20px; font-weight: bold; color: #667eea; margin-top: 5px; }
        .maps-button { display: inline-block; background: #4285f4; color: white; padding: 12px 24px; text-decoration: none; border-radius: 5px; margin: 20px 0; }
        .footer { text-align: center; margin-top: 20px; font-size: 12px; color: #666; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🛬 CanSat Landing Detected!</h1>
            <p>Mission: {{team_id}}</p>
            <p>Time: {{email_sent_at}}</p>
        </div>
        
        <div class="content">
            <h2>📊 Landing Telemetry</h2>
            
            <div class="data-grid">
                <div class="data-item">
                    <div class="data-label">Altitude</div>
                    <div class="data-value">{{altitude}} m</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Flight State</div>
                    <div class="data-value">{{flight_state}} ({{mode}})</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Temperature</div>
                    <div class="data-value">{{temperature}} °C</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Pressure</div>
                    <div class="data-value">{{pressure}} hPa</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Battery</div>
                    <div class="data-value">{{battery}} %</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Signal (RSSI)</div>
                    <div class="data-value">{{rssi}} dBm</div>
                </div>
            </div>
            
            <h3>📍 Landing Location</h3>
            <div class="data-grid">
                <div class="data-item">
                    <div class="data-label">Latitude</div>
                    <div class="data-value">{{latitude}}</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Longitude</div>
                    <div class="data-value">{{longitude}}</div>
                </div>
                <div class="data-item">
                    <div class="data-label">GPS Altitude</div>
                    <div class="data-value">{{gps_altitude}} m</div>
                </div>
                <div class="data-item">
                    <div class="data-label">Satellites</div>
                    <div class="data-value">{{satellites}}</div>
                </div>
            </div>
            
            <center>
                <a href="{{maps_link}}" class="maps-button">📍 View on Google Maps</a>
            </center>
            
            <div class="footer">
                <p>This is an automated notification from the CanSat Ground Control Station</p>
                <p>Packet #{{packet_count}} | Mission Time: {{timestamp}}</p>
            </div>
        </div>
    </div>
</body>
</html>
```

4. Click **Save** and note the **Template ID**

### Step 4: Get Public Key

1. In EmailJS dashboard, go to **Account** → **General**
2. Find your **Public Key** section
3. Copy the public key (starts with a random string)

### Step 5: Configure GCS Environment

1. In your GCS project root, copy `.env.local.example` to `.env.local`:
   ```bash
   cp .env.local.example .env.local
   ```

2. Edit `.env.local` and add your credentials:
   ```env
   NEXT_PUBLIC_EMAILJS_SERVICE_ID=service_xxxxxxx
   NEXT_PUBLIC_EMAILJS_TEMPLATE_ID=template_xxxxxxx
   NEXT_PUBLIC_EMAILJS_PUBLIC_KEY=xxxxxxxxxxxx
   ```

3. Save the file

### Step 6: Restart Development Server

```bash
npm run dev
```

The EmailJS service will automatically initialize when the GCS loads.

## 📧 Email Template Variables

Your email template has access to these variables from the telemetry data:

| Variable | Description | Example |
|----------|-------------|---------|
| `{{team_id}}` | Team identifier | 2024-ASI-CANSAT-049 |
| `{{timestamp}}` | Mission time | 00:05:30 |
| `{{packet_count}}` | Packet number | 330 |
| `{{altitude}}` | Barometric altitude (m) | 125.5 |
| `{{pressure}}` | Atmospheric pressure (hPa) | 1013.25 |
| `{{temperature}}` | Temperature (°C) | 24.5 |
| `{{voltage}}` | Battery voltage (V) | 7.2 |
| `{{battery}}` | Battery percentage | 85 |
| `{{latitude}}` | GPS latitude | 26.720333 |
| `{{longitude}}` | GPS longitude | 84.303806 |
| `{{gps_altitude}}` | GPS altitude (m) | 124.8 |
| `{{satellites}}` | GPS satellites count | 8 |
| `{{speed}}` | GPS speed (m/s) | 0.5 |
| `{{flight_state}}` | Flight state number | 7 |
| `{{mode}}` | Flight mode | IMPACT |
| `{{rssi}}` | Signal strength (dBm) | -65 |
| `{{maps_link}}` | Google Maps URL | https://maps.google.com/... |
| `{{email_sent_at}}` | Email timestamp | 2025-10-27 14:30:45 |

## ✅ Testing

### Test from Browser Console

1. Open GCS in browser
2. Open Developer Tools (F12)
3. Go to Console tab
4. Run test command:
   ```javascript
   // Import the test function
   const { sendTestEmail } = await import('./lib/email-service')
   
   // Send test email
   await sendTestEmail()
   ```

5. Check your email inbox for the test notification

### Test During Mission

The email will automatically send when:
- **flight_state** equals **7** (landed/impact state)
- Email has not already been sent this session
- EmailJS is properly configured

## 🔧 Troubleshooting

### "Email service not configured"
- Check that all three environment variables are set in `.env.local`
- Restart the dev server after adding variables
- Ensure variable names start with `NEXT_PUBLIC_`

### "Failed to send email"
- Check browser console for detailed error messages
- Verify Service ID, Template ID, and Public Key are correct
- Check EmailJS dashboard → Logs for delivery status
- Ensure you haven't exceeded free tier limit (200/month)

### Gmail "Less Secure Apps" Warning
- Use EmailJS OAuth connection (recommended)
- Or enable 2FA and create an App Password

### Email arrives but formatting is broken
- Check template HTML in EmailJS dashboard
- Verify all `{{variables}}` match the template
- Test with EmailJS's built-in template tester

## 📊 Monitoring

### Check Email Status in GCS

```javascript
// In browser console
const { getEmailJSStatus } = await import('./lib/email-service')
console.log(getEmailJSStatus())
```

### View EmailJS Dashboard

1. Go to https://dashboard.emailjs.com/
2. View **Logs** to see sent emails
3. Check **Usage** for monthly quota

## 🔐 Security Notes

- ✅ **Safe**: Public Key can be exposed in client code
- ✅ **Safe**: Service/Template IDs are not sensitive
- ❌ **Never commit** `.env.local` to git (already in `.gitignore`)
- ✅ Email sending is rate-limited by EmailJS
- ✅ No email credentials stored in code

## 📈 Upgrading

Free tier limitations:
- 200 emails/month
- 1 email per 5 seconds
- 50kb email size

To upgrade:
1. Go to EmailJS dashboard → **Billing**
2. Choose a paid plan
3. Increase limits as needed

## 🆘 Support

- EmailJS Documentation: https://www.emailjs.com/docs/
- EmailJS Support: https://www.emailjs.com/support/
- GCS Issues: Create GitHub issue with `[EmailJS]` prefix

---

**Setup Complete!** 🎉

Your GCS will now automatically send landing notifications when the CanSat touches down.
