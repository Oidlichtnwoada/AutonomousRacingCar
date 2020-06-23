#!/usr/bin/env python
from __future__ import print_function
import grpc
import racecar_simulator_pb2
import racecar_simulator_pb2_grpc

class Client:

  def __init__(self, server_address):
    # Connect to server
    self.channel = grpc.insecure_channel(server_address)
    self.connection = racecar_simulator_pb2_grpc.SimulatorStub(self.channel)

  def __del__(self):
    # Disconnect from server
    self.channel.close()

  def get_server_version(self):
    request = racecar_simulator_pb2.Empty()
    response = self.connection.GetVersion(request)
    server_version = "{}.{}.{}".format(response.version_major,
                                       response.version_minor,
                                       response.version_patch)
    return server_version

if __name__ == '__main__':
  client = Client(server_address='localhost:50051')
  server_version = client.get_server_version()
  print("Simulator version: {}".format(server_version))
