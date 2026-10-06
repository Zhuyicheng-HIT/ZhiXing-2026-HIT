from __future__ import annotations
import time
import uuid
from copy import deepcopy


class ScanExecution:
    terminal = {'TIMED_OUT','FAILED','REJECTED','CANCELED','CANCELLED','UNOBSERVABLE','PAUSED'}

    def __init__(self):
        self.active = None
        self.started_monotonic = None
        self.accepted_monotonic = None

    def begin(self, command, attempt=1):
        now = time.monotonic()
        command.update(execution_attempt=attempt,attempt_id=str(uuid.uuid4()))
        self.active = dict(command_id=command['command_id'],mission_epoch=command['mission_epoch'],
            attempt_id=command['attempt_id'],execution_attempt=attempt,status='ISSUED',
            executor_session=None,reason_code=None,retry_available=False,hardware_authorized=False)
        self.started_monotonic = now
        self.accepted_monotonic = None

    def refresh(self, command):
        if not self.active or self.active['status'] in self.terminal:
            return
        now = time.monotonic()
        elapsed = now-(self.accepted_monotonic if self.accepted_monotonic is not None else self.started_monotonic)
        limit = command['execution_timeout_s'] if self.accepted_monotonic is not None else command['accept_before']-command['issued_at']
        self.active['elapsed_wall_s'] = max(0,elapsed)
        if elapsed>limit:
            self.active.update(status='TIMED_OUT',reason_code='EXECUTION_TIMEOUT' if self.accepted_monotonic is not None else 'ACCEPTANCE_TIMEOUT',retry_available=True)

    def validate_attempt(self, payload, command):
        for name in ('vehicle_id','command_id','mission_epoch','plan_revision','point_revision'):
            if payload.get(name)!=command[name]:
                raise ValueError('Execution identity mismatch: '+name)
        attempt = payload.get('attempt_id')
        if attempt is not None and attempt!=command['attempt_id']:
            raise ValueError('Stale execution attempt')
        if command['execution_attempt']>1 and attempt!=command['attempt_id']:
            raise ValueError('Retried observation requires attempt_id')

    def receive(self, payload, command):
        self.validate_attempt(payload,command)
        self.refresh(command)
        if payload.get('attempt_id')!=command['attempt_id']:
            raise ValueError('Execution status requires attempt_id')
        session = payload.get('executor_session')
        if not isinstance(session,str) or not session or len(session)>128:
            raise ValueError('Execution status requires bounded executor_session')
        if self.active['executor_session'] not in (None,session):
            raise ValueError('Execution attempt owned by another session')
        status = payload.get('status')
        if status not in {'ACCEPTED','RUNNING','FAILED','REJECTED','CANCELED','UNOBSERVABLE'}:
            raise ValueError('Unsupported execution status')
        if self.active['status'] in self.terminal:
            if self.active['status']==status and self.active['executor_session']==session:
                return dict(accepted=True,duplicate=True,completed=False)
            raise ValueError('Terminal attempt must be retried explicitly')
        if status=='RUNNING' and self.active['status'] not in ('ACCEPTED','RUNNING'):
            raise ValueError('Execution must be accepted before running')
        if status=='ACCEPTED' and self.active['status']=='RUNNING':
            return dict(accepted=True,duplicate=True,completed=False)
        if status in self.terminal:
            reason = payload.get('reason_code')
            if not isinstance(reason,str) or not reason or len(reason)>256:
                raise ValueError('Failure requires bounded reason_code')
        else:
            reason = None
        if self.accepted_monotonic is None and status=='ACCEPTED':
            self.accepted_monotonic = time.monotonic()
        duplicate = self.active['status']==status
        self.active.update(status=status,executor_session=session,reason_code=reason,
            retry_available=status in self.terminal)
        return dict(accepted=True,duplicate=duplicate,completed=False)

    def validate_result(self, payload, command):
        self.validate_attempt(payload,command)
        self.refresh(command)
        if self.active['status'] in self.terminal or self.active['status']=='PAUSED':
            raise ValueError('Expired or failed attempt cannot complete; retry original point')
        session = self.active['executor_session']
        if session is not None and payload.get('executor_session')!=session:
            raise ValueError('Observation result must belong to accepted executor session')

    def note_result(self, payload):
        if payload.get('status') in {'FAILED','CANCELLED','PAUSED','UNOBSERVABLE'}:
            status = payload['status']
            self.active.update(status=status,reason_code=payload['reason_code'],retry_available=status in self.terminal)

    def snapshot(self):
        return deepcopy(self.active)
