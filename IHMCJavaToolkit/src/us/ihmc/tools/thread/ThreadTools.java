package us.ihmc.tools.thread;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.commons.io.IOUtils;
import org.apache.commons.lang3.SystemUtils;

import us.ihmc.tools.io.StreamGobbler;

public class ThreadTools
{
   public static final int REASONABLE_WAITING_SLEEP_DURATION_MS = 10;

   public static void sleepSeconds(double secondsToSleep)
   {
      try
      {
         Thread.sleep((long) (secondsToSleep * 1000));
      }
      catch (InterruptedException e)
      {
      }
   }
   
   public static void sleep(long milliseconds)
   {
      try
      {
         Thread.sleep(milliseconds);
      }
      catch (InterruptedException ex)
      {
      }
   }
   
   public static void sleep(int milliseconds, int nanoseconds)
   {
      try
      {
         Thread.sleep(milliseconds, nanoseconds);
      }
      catch (InterruptedException ex)
      {
      }
   }

   public static void sleepForever()
   {
      while (true)
      {
         ThreadTools.sleep(1000);
      }
   }
   
   public static void startAThread(Runnable runnable, String threadName)
   {
      Thread newThread = new Thread(runnable, threadName);
      newThread.start();
   }

   public static void startAsDaemon(Runnable daemonThreadRunnable, String threadName)
   {
      Thread daemonThread = new Thread(daemonThreadRunnable, threadName);
      daemonThread.setDaemon(true);
      daemonThread.start();
   }

   public static void waitUntilNextMultipleOf(long waitMultipleMS) throws InterruptedException
   {
      waitUntilNextMultipleOf(waitMultipleMS, 0);
   }

   public static void waitUntilNextMultipleOf(long waitMultipleMS, long moduloOffset) throws InterruptedException
   {
      long startTime = System.currentTimeMillis();
      long numberOfMultiplesThusFar = (startTime - moduloOffset) / waitMultipleMS;
      long endTime = (numberOfMultiplesThusFar + 1) * waitMultipleMS + moduloOffset;
      waitUntil(endTime);
   }

   public static void waitUntil(long endTime) throws InterruptedException
   {
      while (true)
      {
         if (endTime <= System.currentTimeMillis())
            break;
         Thread.sleep(REASONABLE_WAITING_SLEEP_DURATION_MS);
      }
   }

   
   public static ThreadFactory getNamedThreadFactory(final String name)
   {
      return new ThreadFactory()
      {
         private final AtomicInteger threadNumber = new AtomicInteger(1);

         @Override
         public Thread newThread(Runnable r)
         {
            Thread t = new Thread(r, name + "-thread-" + threadNumber.getAndIncrement());

            if (t.isDaemon())
               t.setDaemon(false);
            if (t.getPriority() != Thread.NORM_PRIORITY)
               t.setPriority(Thread.NORM_PRIORITY);

            return t;
         }
      };
   }
   
   public static String getBaseClassName()
   {
      StackTraceElement[] stack = Thread.currentThread().getStackTrace();
      String className = stack[stack.length - 1].getClassName();
      return className;
   }
   
   public static String getBaseSimpleClassName()
   {
      String baseClassName = getBaseClassName();
      int lastDotIndex = baseClassName.lastIndexOf('.');
      String simpleClassName = baseClassName.substring(lastDotIndex + 1);
      return simpleClassName;
   }

   public static void interruptLiveThreadsExceptThisOneContaining(String stringToContain)
   {
      Map<Thread, StackTraceElement[]> allStackTraces = Thread.getAllStackTraces();
      Set<Thread> threadSet = allStackTraces.keySet();
      
      for (Thread thread : threadSet)
      {
         if (thread.isAlive() && thread != Thread.currentThread())
         {
            if (thread.getName().contains(stringToContain))
            {
//               System.out.println("Interrupting thread " + thread.getName());
               thread.interrupt();
            }
         }
      }
   }
   
   public static void interruptAllAliveThreadsExceptThisOne()
   {
      Map<Thread, StackTraceElement[]> allStackTraces = Thread.getAllStackTraces();
      Set<Thread> threadSet = allStackTraces.keySet();
      
      for (Thread thread : threadSet)
      {
         if (thread.isAlive() && thread != Thread.currentThread())
         {
            thread.interrupt();
         }
      }
   }
   
   public static ExecutorService executeWithTimeout(String threadName, Runnable runnable, long timeout, TimeUnit timeUnit)
   {
      ExecutorService executor = Executors.newSingleThreadExecutor(getNamedThreadFactory(threadName));
      executor.execute(runnable);
      executor.shutdown();
      try
      {
         executor.awaitTermination(timeout, timeUnit);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
      
      return executor;
   }

   public static ScheduledFuture<?> scheduleWithFixeDelayAndTimeLimit(String threadName, final Runnable runnable, long initialDelay, long delay, TimeUnit timeUnit, final long timeLimit)
   {
      ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(getNamedThreadFactory(threadName));
      final ScheduledFuture<?> handle = scheduler.scheduleWithFixedDelay(runnable, initialDelay, delay, timeUnit);
      ScheduledFuture<?> handleKiller = scheduler.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            handle.cancel(true);
         }
      }, timeLimit, timeUnit);
      
      return handleKiller;
   }

   public static ScheduledFuture<?> scheduleWithFixedDelayAndIterationLimit(String threadName, final Runnable runnable, long initialDelay, final long delay, final TimeUnit timeUnit, final int iterations)
   {
      final AtomicInteger counter = new AtomicInteger();
      ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(2, getNamedThreadFactory(threadName));
      final ScheduledFuture<?> handle = scheduler.scheduleWithFixedDelay(new Runnable()
      {
         @Override
         public void run()
         {
            if(counter.get() < iterations)
            {
               runnable.run();
               counter.incrementAndGet();
            }
         }
      }, initialDelay, delay, timeUnit);
      
      ScheduledFuture<?> handleKiller = scheduler.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            while(counter.get() < iterations)
            {
               sleep(TimeUnit.MILLISECONDS.convert(delay, timeUnit));
            }
            handle.cancel(true);
         }
      }, 0, timeUnit);
      
      return handleKiller;
   }

}
