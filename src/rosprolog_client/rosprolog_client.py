import json

import rclpy

from json_prolog_msgs import srv
from rclpy.node import Node
from rclpy.client import Client


class PrologException(Exception):
    pass


class PrologQuery(object):
    def __init__(
        self,
        query_str: str,
        simple_query_srv: Client,
        next_solution_srv: Client,
        finish_srv: Client,
        node: Node,
        iterative=True,
    ):
        """
        This class wraps around the different rosprolog services to provide a convenient python interface.
        :type query_str: str
        :type simple_query_srv: rclpy.client.Client
        :type next_solution_srv: rclpy.client.Client
        :type finish_srv: rclpy.client.Client
        :type node: rclpy.node.Node
        :param iterative: if False, all solutions will be calculated by rosprolog during the first service call
        :type iterative: bool
        """
        self._node = node
        self._simple_query_srv = simple_query_srv
        self._next_solution_srv = next_solution_srv
        self._finish_query_srv = finish_srv

        self._finished = False
        self._query_id = None

        request = srv.PrologQuery.Request()
        request.id = self.get_id()
        request.query = query_str
        request.mode = (1 if iterative else 0).to_bytes(1, "big")

        future_response = self._simple_query_srv.call_async(request)
        rclpy.spin_until_future_complete(self._node, future_response)
        result = future_response.result()
        if not result.ok:
            raise PrologException("Prolog query failed: {}".format(result.message))

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.finish()

    def solutions(self):
        """
        :rtype: Iterator[dict]
        """
        try:
            while not self._finished:
                request = srv.PrologNextSolution.Request()
                request.id = self.get_id()

                future_response = self._next_solution_srv.call_async(request)
                rclpy.spin_until_future_complete(self._node, future_response)
                next_solution = future_response.result()

                if next_solution.status == srv.PrologNextSolution.Response.OK:
                    yield self._json_to_dict(next_solution.solution)
                elif next_solution.status == srv.PrologNextSolution.Response.WRONG_ID:
                    raise PrologException(
                        "Query id {} invalid. Maybe another process terminated our query?".format(
                            self.get_id()
                        )
                    )
                elif (
                    next_solution.status == srv.PrologNextSolution.Response.QUERY_FAILED
                ):
                    raise PrologException(
                        "Prolog query failed: {}".format(next_solution.solution)
                    )
                elif (
                    next_solution.status == srv.PrologNextSolution.Response.NO_SOLUTION
                ):
                    break
                else:
                    raise PrologException(
                        "Unknown query status {}".format(next_solution.status)
                    )
        finally:
            self.finish()

    def finish(self):
        if not self._finished:
            try:
                request = srv.PrologFinish.Request()
                request.id = self.get_id()
                future_response = self._finish_query_srv.call_async(request)
                rclpy.spin_until_future_complete(self._node, future_response)
            finally:
                self._finished = True

    def get_id(self):
        """
        :rtype: str
        """
        if self._query_id is None:
            self._query_id = "PYTHON_QUERY_{}".format(
                self._node.get_clock().now().nanoseconds
            )
        return self._query_id

    def _json_to_dict(self, json_text):
        """
        :type json_text: str
        :rtype: dict
        """
        return json.loads(json_text)


class Prolog(object):
    def __init__(
        self, node: Node, name_space="rosprolog", timeout=None, wait_for_services=True
    ):
        """
        :type name_space: str
        :param timeout: Amount of time in seconds spend waiting for rosprolog to become available.
        :type timeout: int
        """
        self.node = node
        self.name_space = name_space
        self._simple_query_srv = self.node.create_client(
            srv.PrologQuery, f"{name_space}/query"
        )
        self._next_solution_srv = self.node.create_client(
            srv.PrologNextSolution, f"{name_space}/next_solution"
        )
        self._finish_query_srv = self.node.create_client(
            srv.PrologFinish, f"{name_space}/finish"
        )
        if wait_for_services:
            self.wait_for_service(timeout)

    def query(self, query_str):
        """
        Returns an Object which asks rosprolog for one solution at a time.
        :type query_str: str
        :rtype: PrologQuery
        """
        return PrologQuery(
            query_str,
            simple_query_srv=self._simple_query_srv,
            next_solution_srv=self._next_solution_srv,
            finish_srv=self._finish_query_srv,
            node=self.node,
        )

    def once(self, query_str):
        """
        Call rosprolog once and finished it.
        :type query_str: str
        :rtype: list
        """
        q = None
        try:
            q = PrologQuery(
                query_str,
                simple_query_srv=self._simple_query_srv,
                next_solution_srv=self._next_solution_srv,
                finish_srv=self._finish_query_srv,
                node=self.node,
            )
            return next(q.solutions())
        except StopIteration:
            return []
        finally:
            if q is not None:
                q.finish()

    def all_solutions(self, query_str):
        """
        Requests all solutions from rosprolog, this might take a long time!
        :type query_str: str
        :rtype: list
        """
        return list(
            PrologQuery(
                query_str,
                iterative=False,
                simple_query_srv=self._simple_query_srv,
                next_solution_srv=self._next_solution_srv,
                finish_srv=self._finish_query_srv,
                node=self.node,
            ).solutions()
        )

    def wait_for_service(self, timeout=None):
        """
        Wait for services to be ready.
        """
        self.node.get_logger().info(f"waiting for {self.name_space} services")
        self._simple_query_srv.wait_for_service(timeout_sec=timeout)
        self._next_solution_srv.wait_for_service(timeout_sec=timeout)
        self._finish_query_srv.wait_for_service(timeout_sec=timeout)

        if (
            self._finish_query_srv.service_is_ready()
            and self._simple_query_srv.service_is_ready()
            and self._next_solution_srv.service_is_ready()
        ):
            self.node.get_logger().info(f"{self.name_space} services ready")
            return

        self.node.get_logger().warn(
            f"not all {self.name_space} services were ready when timeout was reached!"
        )
