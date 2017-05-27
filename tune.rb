# frozen_string_literal: true

require 'csv'
require 'English'
require 'json'
require 'timeout'
require 'tmpdir'

require 'cross_entropy'

DISTANCE_WEIGHT = -1
TOTAL_ABSOLUTE_CTE_WEIGHT = 0

#
# Run a command and capture the stdout and stderr.
#
def run(*args, **options)
  Dir.mktmpdir do |tmp|
    out_pathname = File.join(tmp, 'out')
    err_pathname = File.join(tmp, 'err')
    options[:out] = out_pathname
    options[:err] = err_pathname
    options[:in] = '/dev/null'

    command = args.map(&:to_s)
    Process.waitpid(Process.spawn(*command, **options))

    [
      $CHILD_STATUS.exitstatus,
      File.read(out_pathname),
      File.read(err_pathname)
    ]
  end
end

#
# The MPC sim does not support reset, so automate clicking the buttons.
# Note: this only works on Mac.
#
# 1. This assumes that the sim is already running, and that it starts on the
# default loading screen with project 1 selected.
#
# 2. You need to `brew install cliclick`.
#
# 3. You may need to allow your terminal application Accessibility privileges:
# go to System Preferences > Security & Privacy > Privacy > Accessibility and
# then tick the box for your terminal application. (It doesn't look like this
# is required for the standard Mac terminal application, but if you use one,
# such as iTerm, it is.)
#
# 4. Do not move the mouse while the tuning script is running, just to be safe.
#

CLICK_DELAY = 500 # ms

def osascript(*commands)
  arguments = commands.map { |command| ['-e', command] }.flatten
  status, out, err = run('osascript', *arguments)
  raise "osascript failed: #{status}: #{err}" unless status.zero?
  out
end

def find_sim_position
  out = osascript(
    'tell application "self_driving_car_nanodegree_program" to activate',
    'tell application "System Events" to' \
      ' tell application process "self_driving_car_nanodegree_program" to' \
      ' get position of window 1'
  )
  position = out.split(',').map(&:strip).map(&:to_i)
  raise "bad position: #{position}" unless position.size == 2
  position
end

def cliclick(*commands)
  status, out, err = run('cliclick', '-w', CLICK_DELAY.to_s, '-r', *commands)
  raise "cliclick failed: #{status}: #{err}" unless status.zero?
  out
end

def check_color_is_green(color)
  color = color.split.map(&:to_i)
  raise "bad color: #{color}" unless color.size == 3
  unless color[0] < 100 && color[1] > 250 && color[2] < 100 # RGB
    STDERR.puts "Unexpected MPC line color: #{color} (wrong project?)"
  end
end

def start_sim
  position = find_sim_position

  # Click the left arrow to get project 5, and then check that we're on project
  # 5 by looking for the green MPC line, then click the button.
  mpc_line_color = cliclick(
    "c:#{[position[0] + 110, position[1] + 320].join(',')}",
    'cp:+220,-57',
    'c:+210,+85'
  )

  check_color_is_green(mpc_line_color)
  sleep 5
end

def stop_sim
  position = find_sim_position
  top_left = [position[0] + 20, position[1] + 30] # to focus
  cliclick "c:#{top_left.join(',')}", 'kp:esc'
  sleep 2
end

#
# Automate running the MPC application and collecting results.
#
MAX_RUNTIME = 90
RUN_TIMEOUT = MAX_RUNTIME + 5

def strip_non_json(out)
  # In particular, we need to get rid of the message from IPOPT.
  out.lines.grep(/^{/).join
end

def run_and_log(csv, *params)
  start_sim
  status, out, _err = Timeout.timeout(RUN_TIMEOUT) do
    run('build/mpc', *params)
  end
  if [0, 1].member?(status)
    crashed = status == 1
    stats = JSON.parse(strip_non_json(out))
    csv << [
      *params,
      crashed,
      stats['runtime'], stats['distance'], stats['total_absolute_cte']
    ]
    DISTANCE_WEIGHT * stats['distance'] +
      TOTAL_ABSOLUTE_CTE_WEIGHT * stats['total_absolute_cte']
  else
    # The run failed.
    csv << params
    Float::INFINITY
  end
rescue Timeout::Error
  STDERR.puts 'WARNING: Simulator timed out'
  csv << params
  Float::INFINITY
ensure
  stop_sim
end

#
# Cross Entropy Method search
#

COLUMNS = %w(
  max_runtime dt reference_speed cte_weight epsi_weight v_weight
  delta_weight throttle_weight delta_gap_weight throttle_gap_weight
  crashed runtime distance total_absolute_cte
).freeze

def cross_entropy
  # Our initial guess at the optimal solution.
  initial_weights = NMath.log(NArray[1, 1, 1, 1, 1, 1])
  initial_weights_stddev = NArray[3, 3, 3, 3, 3, 3]

  # Fix some parameters.
  # Note that we can fix one of the weights at 1, since they are all relative.
  dt = 0.05
  reference_speed = 50.0
  cte_weight = 1.0

  # Can run multiple trials per sample to make it less likely that we 'get
  # lucky'.
  num_trials = 1

  CSV(STDOUT) do |csv|
    csv << COLUMNS

    # Set up the problem. These are the CEM parameters.
    problem = CrossEntropy::ContinuousProblem.new(
      initial_weights, initial_weights_stddev.dup
    )
    problem.num_samples = 100
    problem.num_elite = 10
    problem.max_iters = 20

    # Objective function (to be minimized).
    problem.to_score_sample do |params|
      weights = NMath.exp(params).to_a
      Array.new(num_trials) do
        run_and_log(csv, MAX_RUNTIME, dt, reference_speed, cte_weight, *weights)
      end.sum / num_trials
    end

    # Do some smoothing when updating the parameters based on new samples.
    # This isn't strictly required, but I find it often helps convergence.
    # Log the results
    smooth = 0.5
    problem.to_update do |new_mean, new_stddev|
      STDERR.puts new_mean.inspect
      STDERR.puts new_stddev.inspect
      smooth_mean = smooth * new_mean + (1 - smooth) * problem.param_mean
      smooth_stddev = smooth * new_stddev +
                      (1 - smooth) * problem.param_stddev
      [smooth_mean, smooth_stddev]
    end

    problem.solve
    STDERR.puts problem.param_mean.inspect
  end
end

cross_entropy
