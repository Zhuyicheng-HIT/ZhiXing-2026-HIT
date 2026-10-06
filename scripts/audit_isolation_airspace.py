from __future__ import annotations
import json
from fleet_core import ROOT, build_plan
from isolation_airspace import audit_isolation


if __name__=='__main__':
    report = audit_isolation(build_plan())
    (ROOT/'validation/isolation_airspace_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
