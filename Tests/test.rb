#!/usr/bin/env ruby
require 'open3'

# Runs the Python test runner and reports results.
# Usage: ruby Tests/test.rb

runner = File.join(__dir__, 'test_runner.py')
cmd = ['python', '-u', runner]

stdout, stderr, status = Open3.capture3(*cmd)

# Print captured output
puts stdout unless stdout.nil? || stdout.empty?
warn stderr unless stderr.nil? || stderr.empty?

if status.success?
  puts "TEST_RUNNER: PASS"
  exit 0
else
  puts "TEST_RUNNER: FAIL (exit #{status.exitstatus})"
  exit(status.exitstatus || 1)
end
