#pragma once

// =============================================================================
// portal.h  –  Hotspot/Config-Portal (AP + Webserver + Captive Portal)
//
// Startet einen offenen Access Point ("Morse"), einen kleinen Webserver unter
// http://192.168.4.1 und einen DNS-Responder (Captive Portal), damit das Handy
// die Konfigurationsseite automatisch öffnet.
// =============================================================================

void portalStart();
void portalStop();
bool portalIsActive();
