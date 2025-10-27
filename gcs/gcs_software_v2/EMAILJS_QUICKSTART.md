# 🚀 EmailJS Quick Start - CanSat GCS

## Step 1: Get EmailJS Credentials (5 min)
1. Sign up at **https://www.emailjs.com**
2. Create email service → Note **Service ID**
3. Create template (use template from EMAILJS_SETUP.md) → Note **Template ID**
4. Copy **Public Key** from Account settings

## Step 2: Configure GCS
```bash
cp .env.local.example .env.local
```

Edit `.env.local`:
```env
NEXT_PUBLIC_EMAILJS_SERVICE_ID=service_abc123
NEXT_PUBLIC_EMAILJS_TEMPLATE_ID=template_xyz789
NEXT_PUBLIC_EMAILJS_PUBLIC_KEY=your_public_key
```

## Step 3: Start GCS
```bash
npm run dev
```

## Step 4: Test (Optional)
Open browser console (F12):
```javascript
const { sendTestEmail } = await import('./lib/email-service')
await sendTestEmail()
```

## ✅ Done!
Emails will automatically send when **flight_state === 7** (landing detected)

---

**Full setup guide:** See `EMAILJS_SETUP.md`
