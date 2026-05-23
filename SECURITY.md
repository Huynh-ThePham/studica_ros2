# Security Policy

This repository controls physical motors. Treat security issues as safety issues.

## Reporting

Before public release, replace this section with the official project security
contact. Do not publish exploit details until the maintainer has acknowledged
and triaged the report.

## Scope

Relevant issues include:

- unauthenticated network command exposure outside a trusted robot LAN,
- packet parsing bugs that can crash the VMX daemon,
- failure to stop motors on timeout or process shutdown,
- unsafe default deployment settings.

## Deployment Guidance

- Run the UDP command port only on a physically trusted robot LAN.
- Do not expose UDP/15000 to campus, lab-wide or Internet-routed networks.
- Add firewall rules or a VPN if the robot must operate outside an isolated LAN.
- Use a hardware emergency stop for every mobile robot platform.
