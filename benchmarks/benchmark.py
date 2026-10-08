from configs.scenes_manager import Scene, load_scene
from configs.settings import GRAV_CONST

from concurrent.futures import ProcessPoolExecutor, as_completed
from os import cpu_count

from physics.utils import Vector, Integrator, EulerIntegrator, VerletIntegrator
from physics.bodies import PhysicalObject
from physics.connections import Joint

from dataclasses import dataclass
from typing import List, Tuple

from abc import ABC, abstractmethod

from time import perf_counter


from rich.table import Table
from rich.console import Console
from rich.panel import Panel



@dataclass
class MetricResult(ABC):
    name: str
    def __str__(self):
        metric_dict = self.__dict__
        text_to_display=""
        text_to_display+=f"\n{"#"*20}\nMetric: {self.name}\n{"#"*20}\n"
        for key, value in metric_dict.items():
            if key == "name":
                continue
            text_to_display+=f"{key}: {value}\n"
        text_to_display+=f"{"#"*20}\n"
        return text_to_display
    def __rich__(self):
        table = Table.grid(padding=(0, 2))

        for key, value in self.__dict__.items():
            if key == "name":
                continue

            if isinstance(value, float):
                value = f"{value:.4f}"
            if isinstance(value, list):
                value = [f"{v:.4f}" if isinstance(v, float) else str(v) for v in value]
                value = ", ".join(value)

            table.add_row(key, str(value))

        return Panel(
            table,
            title=f"[bold]{self.name}[/bold]",
            border_style="cyan"
            ,width=60
        )

class Metric(ABC):
    @abstractmethod
    def start(self, *args, **kwargs):...

    @abstractmethod
    def process(self, *args, **kwargs):...

    @abstractmethod
    def end(self, *args, **kwargs) -> MetricResult:...

@dataclass
class PerformanceMetricResult(MetricResult):
    name : str
    real_time : float 
    simulated_time : float 
    total_steps : int
    total_substeps: int
    avg_step_time: float
    real_time_factor: float

    def __str__(self):
        text_to_display=""
        text_to_display+=f"\n{"#"*20}\nMetric: {self.name}\n{"#"*20}\n"
        text_to_display+=f"real_time: {self.real_time:.4f}s\n"
        text_to_display+=f"simulated_time: {self.simulated_time:.4f}s\n"
        text_to_display+=f"total_steps: {self.total_steps}\n"
        text_to_display+=f"total_substeps: {self.total_substeps}\n"
        text_to_display+=f"avg_step_time: {self.avg_step_time:.4f}s\n"
        text_to_display+=f"real_time_factor: {self.real_time_factor:.4f}\n"
        text_to_display+=f"{"#"*20}\n"
        return text_to_display

    def __rich__(self):
        table = Table.grid(padding=(0, 2))

        table.add_row("real_time", f"{self.real_time:.4f}s")
        table.add_row("simulated_time", f"{self.simulated_time:.4f}s")
        table.add_row("total_steps", str(self.total_steps))
        table.add_row("total_substeps", str(self.total_substeps))
        table.add_row("avg_step_time", f"{self.avg_step_time:.4f}s")
        table.add_row("real_time_factor", f"{self.real_time_factor:.4f}")

        return Panel(
            table,
            title=f"[bold]{self.name}[/bold]",
            border_style="cyan"
            ,width=60
        )

@dataclass
class PhysicsErrorMetricResult(MetricResult):
    energy_drift: float 
    energy_min : float
    energy_max: float
    trajectory_reference_avg_error:float
    trajectory_reference_endpoint_drift: Tuple[Vector, float]
    joint_constraint_error_max: float
    joint_constraint_error_avg: float
    
    name: str 

    def __str__(self):
        text_to_display=""
        text_to_display+=f"\n{"#"*20}\nMetric: {self.name}\n{"#"*20}\n"
        text_to_display+=f"energy_drift: {self.energy_drift*100:.4f}%\n"
        text_to_display+=f"energy_min: {self.energy_min:.4f} J\n"
        text_to_display+=f"energy_max: {self.energy_max:.4f} J\n"
        text_to_display+=f"trajectory_reference_avg_error: {self.trajectory_reference_avg_error:.4f} px\n"
        text_to_display+=f"trajectory_reference_endpoint_drift: {self.trajectory_reference_endpoint_drift[0]} (length: {self.trajectory_reference_endpoint_drift[1]:.4f} px)\n"
        text_to_display+=f"joint_constraint_error_max: {self.joint_constraint_error_max:.4f} px\n"
        text_to_display+=f"joint_constraint_error_avg: {self.joint_constraint_error_avg:.4f} px\n"
        text_to_display+=f"{"#"*20}\n"
        return text_to_display

    def __rich__(self):
        table = Table.grid(padding=(0, 2))

        table.add_row("energy_drift", f"{self.energy_drift*100:.4f}%")
        table.add_row("energy_min", f"{self.energy_min:.4f} J")
        table.add_row("energy_max", f"{self.energy_max:.4f} J")
        table.add_row("trajectory_reference_avg_error", f"{self.trajectory_reference_avg_error:.4f} px")
        table.add_row("trajectory_reference_endpoint_drift", f"{self.trajectory_reference_endpoint_drift[0]} (length: {self.trajectory_reference_endpoint_drift[1]:.4f} px)")
        table.add_row("joint_constraint_error_max", f"{self.joint_constraint_error_max:.4f} px")
        table.add_row("joint_constraint_error_avg", f"{self.joint_constraint_error_avg:.4f} px")

        return Panel(
            table,
            title=f"[bold]{self.name}[/bold]",
            border_style="cyan"
            ,width=60
        )
@dataclass
class PhysicsErrorMetric(Metric):
    def start(self, **kwargs):
        self.objects : List[PhysicalObject] = kwargs["objects"]
        self.joints : List[Joint] = kwargs["joints"]

        self.trajectory_arr: List[List[Vector]] = [[]]
        self.reference_trajectory: List[List[Vector]] = kwargs["reference_trajectory"]

        self.joint_constraint_arr : List[List[float]] = [[]]
        self.joint_constraint_reference : List[List[float]] = kwargs["joint_constraint_reference"]

        self.step=0
        self.start_energy=0
        for object in self.objects:
            self.start_energy+=(1/2 * object.mass * object.velocity.get_length()**2) + (object.mass*GRAV_CONST*object.pos.y)
        for joint in self.joints:
           summ_elong=0
           for sector in joint.sectors:
               summ_elong+= sector.elong
               self.start_energy+=1/2*sector.elong**2 * sector.stiffnes_cf
        self.energy_min = self.start_energy
        self.energy_max = self.start_energy

    def process(self, **kwargs):
        energy=0
        i=0
        for object in self.objects:
            energy+=(1/2 * object.mass * object.velocity.get_length()**2) + (object.mass*GRAV_CONST*object.pos.y)
            try:
                self.trajectory_arr[i].append(object.pos)
            except IndexError:
                self.trajectory_arr.append([object.pos])
            i+=1
        i=0
        for joint in self.joints:
            summ_elong=0
            for sector in joint.sectors:
                summ_elong+=sector.elong
                energy+=1/2*sector.elong**2 * sector.stiffnes_cf
            try:
                self.joint_constraint_arr[i].append(summ_elong)
            except IndexError:
                self.joint_constraint_arr.append([summ_elong])
            i+=1
        self.energy=energy
            
        if energy < self.energy_min:
            self.energy_min=energy
        if energy > self.energy_max:
            self.energy_max=energy
    def end(self):
        energy_drift = (self.energy-self.start_energy)/self.start_energy
        avg_error_list : List[float]= []
        endpoint_drift_list : List[Vector]= []

        for i in range(0, len(self.trajectory_arr)):
            avg_error=0
            for j in range(0, len(self.trajectory_arr[i])):
                avg_error += (self.trajectory_arr[i][j] - self.reference_trajectory[i][j]).get_length()
            avg_error /= len(self.trajectory_arr[i])
            avg_error_list.append(avg_error)

            endpoint_drift = self.trajectory_arr[i][-1]-self.reference_trajectory[i][-1]
            endpoint_drift_list.append(endpoint_drift)
        avg_err_result = sum(avg_error_list)/len(avg_error_list)
        sum_endpoints_drift=Vector(0,0)
        for endpoint_drift in endpoint_drift_list:
            sum_endpoints_drift+=endpoint_drift
        avg_endpoint_drift : Vector =sum_endpoints_drift/len(endpoint_drift_list)
        avg_endpoint_drift_length=sum([endpoint_drift.get_length() for endpoint_drift in endpoint_drift_list])/len(endpoint_drift_list)

        avg_constraint_err_list: List[float]=[]
        constraint_err_max = 0
        for i in range(0, len(self.joint_constraint_arr)): 
            avg_constraint_err=0
            for j in range(0, len(self.joint_constraint_arr[i])):
                err=abs(abs(self.joint_constraint_arr[i][j])-abs(self.joint_constraint_reference[i][j]))
                avg_constraint_err+=err
                if err > constraint_err_max:
                    constraint_err_max = err
            avg_constraint_err_list.append(avg_constraint_err/len(self.joint_constraint_reference[i]))

        constraint_err_avg = sum(avg_constraint_err_list)/len(avg_constraint_err_list)

        return PhysicsErrorMetricResult(name="Physics Error",
                                 energy_drift=energy_drift,
                                 energy_min=self.energy_min,
                                 energy_max=self.energy_max,
                                 trajectory_reference_avg_error=avg_err_result,
                                 trajectory_reference_endpoint_drift=[avg_endpoint_drift, avg_endpoint_drift_length],
                                 joint_constraint_error_max=constraint_err_max,
                                 joint_constraint_error_avg=constraint_err_avg
                                )
        

        

class PerformanceMetric(Metric):
    def start(self, **kwargs):
        self.start_time=0
        self.step=0
        self.substeps=0
        self.substeps_const=kwargs["substeps"]
        self.physics_dt=kwargs["physics_dt"]
        self.previous_time=perf_counter()
        self.total_step_time=0

    def process(self, **kwargs):
        current_time=perf_counter()
        step_time = kwargs["step_time"]
        if self.start_time == 0:
            self.start_time= current_time-step_time
        self.total_step_time+=step_time
        self.step+=1
        self.substeps+=self.substeps_const

    def end(self):
        current_time = perf_counter()
        real_time = current_time-self.start_time
        simulated_time = self.physics_dt*self.step
        rtf=simulated_time/real_time
        avg_step_time = self.total_step_time/self.step
        return PerformanceMetricResult(name="Performance", real_time=real_time, simulated_time=simulated_time, total_steps=self.step, total_substeps=self.substeps,avg_step_time=avg_step_time, real_time_factor=rtf)
    

        


class Benchmark():
    def __init__(self, scene_name : str, steps: int, physics_dt : float, metrics: List[Metric], substeps: int, reference_to_target_substep_factor: float = 1, integrator_type: Integrator = EulerIntegrator):
        try:
            self.scene = load_scene(scene_name, path="./benchmarks/scenes/", substeps=substeps, used_integrator=integrator_type)
        except FileNotFoundError:
            print("Save does not exist. Create one and load using its name.")
        self.steps = steps
        self.physics_dt = physics_dt
        self.reference_to_target_substep_factor=reference_to_target_substep_factor
        self.metrics : List[Metric] = metrics

    def simulate(self):
        # print(f"\nSimulating scene: {self.scene.name}")
        for metric in self.metrics:
            if isinstance(metric, PerformanceMetric):
                metric.start(physics_dt=self.physics_dt, substeps=self.scene.substeps)
            if isinstance(metric, PhysicsErrorMetric):
                # print("Initiating reference simulation to analyse trajectory draft.")
                start_time= perf_counter()
                objects_list = self.scene.objects
                joints_list= self.scene.joints
                copy_scene= load_scene(self.scene.name, path="./benchmarks/scenes/", substeps=self.scene.substeps*self.reference_to_target_substep_factor, used_integrator=self.scene.used_integrator)
                reference_trajectory=[]
                reference_constraint=[]
                for i in range(self.steps):
                    copy_scene.step(self.physics_dt)
                    j=0
                    for object in copy_scene.objects:
                        try: 
                            reference_trajectory[j].append(object.pos)
                        except IndexError:
                            reference_trajectory.append([object.pos])
                        j+=1
                    j=0 
                    for joint in copy_scene.joints:
                        summ_elong=0
                        for sector in joint.sectors:
                            summ_elong+=sector.elong
                        try: 
                            reference_constraint[j]
                            reference_constraint[j].append(summ_elong)
                        except IndexError:
                            reference_constraint.append([summ_elong])
                        j+=1
                metric.start(objects= objects_list,
                             joints = joints_list,
                             joint_constraint_reference = reference_constraint,
                             reference_trajectory = reference_trajectory
                             )
                run_time=perf_counter()-start_time
                # print(f"Ended in {run_time}s. Simulating scene...")

        for i in range(self.steps):
            start_time=perf_counter()
            self.scene.step(self.physics_dt)
            step_time=perf_counter()-start_time
            for metric in self.metrics:
                metric.process(step_time=step_time)
        result_metrics : List[MetricResult]=[]
        for metric in self.metrics:
            result_metrics.append(metric.end())

        return result_metrics

@dataclass(frozen=True)
class BenchmarkConfig():
    scene_name: str
    steps: int
    physics_dt:float
    metrics: List[Metric]
    substeps: int
    reference_to_target_substep_factor : float=1,
    integrator_type: Integrator = EulerIntegrator


def benchmark_worker(config: BenchmarkConfig):
    benchmark = Benchmark(**config.__dict__)
    result = benchmark.simulate()
    return result


class Sweep():
    def __init__(self, scene_name : str, steps: int | List[int], physics_dt : float | List[float], metrics: List[Metric] , substeps: int | List[int], reference_to_target_substep_factor: float | List[float] = 1, integrator_type: List[Integrator] | Integrator = EulerIntegrator):
        self.max_process = max(1, cpu_count()-2)
        params_lens= [len(param) if isinstance(param, list) else 1 for param in [steps, physics_dt, substeps, reference_to_target_substep_factor, integrator_type]]
        for param in [steps, physics_dt, substeps, reference_to_target_substep_factor, integrator_type]:
            if isinstance(param, list) and len(param) not in params_lens:
                raise ValueError("All list parameters must have the same length.")    

        self.benchmarks_count = max(params_lens)

        self.scene_name = scene_name
        self.steps = steps
        self.physics_dt=physics_dt
        self.metrics=metrics
        self.substeps=substeps
        self.reference_to_target_substep_factor=reference_to_target_substep_factor
        self.integrator_type=integrator_type
    def simulate(self):
        configs : List[BenchmarkConfig]=[]
        for i in range(0, self.benchmarks_count):
            if isinstance(self.steps, list):
                steps = self.steps[i]
            else:
                steps=self.steps
            if isinstance(self.physics_dt, list):
                physics_dt = self.physics_dt[i]
            else:
                physics_dt=self.physics_dt

            if isinstance(self.substeps, list):
                substeps = self.substeps[i]
            else:
                substeps=self.substeps
            if isinstance(self.reference_to_target_substep_factor, list):
                reference_to_target_substep_factor = self.reference_to_target_substep_factor[i]
            else:
                reference_to_target_substep_factor=self.reference_to_target_substep_factor
            if isinstance(self.integrator_type, list):
                integrator_type = self.integrator_type[i]
            else:
                integrator_type=self.integrator_type
            config = BenchmarkConfig(
                scene_name=self.scene_name,
                steps=steps,
                physics_dt=physics_dt,
                metrics=self.metrics,
                substeps=substeps,
                reference_to_target_substep_factor=reference_to_target_substep_factor,
                integrator_type=integrator_type
            )
            configs.append(config)
        with ProcessPoolExecutor(max_workers=min(self.benchmarks_count, self.max_process)) as executor:
            future_to_index = {
                executor.submit(benchmark_worker, config): i
                for i, config in enumerate(configs)
            }

            results = [None] * len(future_to_index)
            for future in as_completed(future_to_index):
                i = future_to_index[future]
                results[i] = future.result()
            return results
            




def main():
    benchmarks : List[Sweep] = []
    #Here you can add some benchmarks using class creator + append.
    console= Console()

    benchmarks.append(Sweep(
        scene_name="pendulum",
        steps=10000,
        physics_dt=1/120,
        reference_to_target_substep_factor= 16,
        metrics=[
                    PerformanceMetric(),
                    PhysicsErrorMetric()
                ],
        substeps= [1,2,4,8, 16, 1, 2, 4, 8, 16],
        integrator_type=[VerletIntegrator(),VerletIntegrator(), VerletIntegrator(), VerletIntegrator(), VerletIntegrator(), EulerIntegrator(), EulerIntegrator(), EulerIntegrator(), EulerIntegrator(), EulerIntegrator()]
    ))


    for bench_count, benchmark in enumerate(benchmarks):
        result_metrics : List[List[MetricResult]] = benchmark.simulate()
        if len(result_metrics) > 1:
            metric_table = Table(title=f"Benchmark №{bench_count} results")
            benchmark_fields = [
                "steps",
                "physics_dt",
                "substeps",
                "reference_to_target_substep_factor",
                "integrator_type"
            ]

            sweep_fields = [
                field for field in benchmark_fields
                if isinstance(getattr(benchmark, field), list)
            ]

            metric_fields = []
            for metrics in result_metrics:
                for metric in metrics:
                    for field in metric.__dict__:
                        if field not in metric_fields and field != "name":
                            metric_fields.append(field)

            for field in sweep_fields + metric_fields:
                metric_table.add_column(field, justify="center")

            for i, metrics in enumerate(result_metrics):
                metric_values = {}
        

                for metric in metrics:
                    metric_values.update(metric.__dict__)

                row = [
                    f"{(getattr(benchmark, field)[i]):.4f}" if isinstance((getattr(benchmark, field)[i]), float) else str((getattr(benchmark, field)[i]))
                    for field in sweep_fields
                ] + [
                    f"{metric_values.get(field, ""):.4f}" if isinstance(metric_values.get(field, ""), float) else str((metric_values.get(field, "")))
                    for field in metric_fields
                    
                ]

                metric_table.add_row(*row)

            console.print(metric_table)
        else:
            for metric in result_metrics:
                for submetric in metric:
                    console.print(submetric)

if __name__ == "__main__":
    main()