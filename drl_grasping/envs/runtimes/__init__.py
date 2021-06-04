try:
    from .real_evaluation_runtime import RealEvaluationRuntimeManual
except Exception as e:
    print(f"RealEvaluationRuntimeManual is disabled - {e}")
